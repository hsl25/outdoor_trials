#include "lwip/tcp.h"
#include <cstring>
#include <string>
#include <stdio.h>
#include "state_manager.hpp"
#include "json_builder.hpp"

extern volatile bool g_estop_active;
static constexpr uint16_t TCP_PORT = 5555;
static constexpr uint8_t  MAX_CMD_LEN = 128;
bool tcp_server_init();
static struct tcp_pcb* g_server_pcb = nullptr;
static struct tcp_pcb* g_client_pcb = nullptr;
static err_t server_accept(void* arg, struct tcp_pcb* newpcb, err_t err);
static err_t server_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err);
static err_t server_sent(void* arg, struct tcp_pcb* tpcb, u16_t len);
static err_t server_poll(void* arg, struct tcp_pcb* tpcb);
static void  server_error(void* arg, err_t err);

static void close_client_connection(struct tcp_pcb* tpcb);
static bool handle_command_and_reply(struct tcp_pcb* tpcb, const char* cmd);


bool tcp_server_init() {
    g_server_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!g_server_pcb) {
        printf("tcp_new_ip_type failed\n");
        return false;
    }

    err_t err = tcp_bind(g_server_pcb, IP_ANY_TYPE, TCP_PORT);
    if (err != ERR_OK) {
        printf("tcp_bind failed: %d\n", err);
        tcp_close(g_server_pcb);
        g_server_pcb = nullptr;
        return false;
    }

    g_server_pcb = tcp_listen_with_backlog(g_server_pcb, 1);
    if (!g_server_pcb) {
        printf("tcp_listen_with_backlog failed\n");
        return false;
    }

    tcp_accept(g_server_pcb, server_accept);

    printf("TCP server listening on port %u\n", TCP_PORT);
    return true;
}


static err_t server_accept(void* arg, struct tcp_pcb* newpcb, err_t err) {
    (void)arg;
    (void)err;

    if (!newpcb) {
        return ERR_VAL;
    }

    
    if (g_client_pcb != nullptr) {
        const char* busy_msg = "{\"error\":\"busy\"}\n";
        tcp_write(newpcb, busy_msg, std::strlen(busy_msg), TCP_WRITE_FLAG_COPY);
        tcp_output(newpcb);
        tcp_close(newpcb);
        return ERR_OK;
    }

    g_client_pcb = newpcb;
    state_manager_set_gui_connected(true);

    printf("Client connected\n");

    tcp_arg(newpcb, nullptr);
    tcp_recv(newpcb, server_recv);
    tcp_sent(newpcb, server_sent);


    tcp_poll(newpcb, server_poll, 4);

    tcp_err(newpcb, server_error);

    return ERR_OK;
}


static err_t server_recv(void* arg, struct tcp_pcb* tpcb, struct pbuf* p, err_t err) {
    (void)arg;

    if (err != ERR_OK) {
        if (p) {
            pbuf_free(p);
        }
        return err;
    }

    
    if (p == nullptr) {
        printf("Client disconnected\n");
        close_client_connection(tpcb);
        return ERR_OK;
    }

    
    tcp_recved(tpcb, p->tot_len);

    char cmd[MAX_CMD_LEN] = {0};

    if (p->tot_len >= MAX_CMD_LEN) {
        pbuf_free(p);

        const char* msg = "{\"error\":\"command too long\"}\n";
        tcp_write(tpcb, msg, std::strlen(msg), TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        return ERR_BUF;
    }

    
    pbuf_copy_partial(p, cmd, p->tot_len, 0);
    pbuf_free(p);

    
    size_t len = std::strlen(cmd);
    while (len > 0 && (cmd[len - 1] == '\n' || cmd[len - 1] == '\r')) {
        cmd[len - 1] = '\0';
        --len;
    }

    printf("Received command: %s\n", cmd);

    if (!handle_command_and_reply(tpcb, cmd)) {
        const char* msg = "{\"error\":\"reply failed\"}\n";
        tcp_write(tpcb, msg, std::strlen(msg), TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        return ERR_MEM;
    }

    return ERR_OK;
}


static err_t server_sent(void* arg, struct tcp_pcb* tpcb, u16_t len) {
    (void)arg;
    (void)tpcb;
    (void)len;
    return ERR_OK;
}


static err_t server_poll(void* arg, struct tcp_pcb* tpcb) {
    (void)arg;
    (void)tpcb;
    return ERR_OK;
}


static void server_error(void* arg, err_t err) {
    (void)arg;
    printf("TCP client error: %d\n", err);

    g_client_pcb = nullptr;
    state_manager_set_gui_connected(false);
}


static void close_client_connection(struct tcp_pcb* tpcb) {
    if (!tpcb) {
        return;
    }

    tcp_arg(tpcb, nullptr);
    tcp_recv(tpcb, nullptr);
    tcp_sent(tpcb, nullptr);
    tcp_poll(tpcb, nullptr, 0);
    tcp_err(tpcb, nullptr);

    err_t err = tcp_close(tpcb);
    if (err != ERR_OK) {
        
        tcp_abort(tpcb);
    }

    if (g_client_pcb == tpcb) {
        g_client_pcb = nullptr;
    }

    state_manager_set_gui_connected(false);
}


static bool handle_command_and_reply(struct tcp_pcb* tpcb, const char* cmd) {
    std::string reply;

    if (std::strcmp(cmd, "GET_STATE") == 0) {
        RoverState state = state_manager_get_snapshot();
        reply = json_build_state(state);
        reply += "\n";
    }
    else if (std::strcmp(cmd, "PING") == 0) {
        reply = "{\"pong\":true}\n";
    }
    else if (std::strcmp(cmd, "ESTOP_ON") == 0) {
        g_estop_active = true;
        reply = "{\"estop\":true}\n";
    }
    else if (std::strcmp(cmd, "ESTOP_OFF") == 0) {
        g_estop_active = false;
        reply = "{\"estop\":false}\n";
    }
    else {
        reply = "{\"error\":\"unknown command\"}\n";
    }

    err_t err = tcp_write(tpcb, reply.c_str(), reply.size(), TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("tcp_write failed: %d\n", err);
        return false;
    }

    err = tcp_output(tpcb);
    if (err != ERR_OK) {
        printf("tcp_output failed: %d\n", err);
        return false;
    }

    return true;
}