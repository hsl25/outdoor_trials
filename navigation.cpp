#include <stdio.h>
#include <math.h>
#include <vector>

#include "navigation.hpp"

// Constructor
Navigation::Navigation() {
    
}

float Navigation:: calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2) {
    int theta_diff = angle2 - angle1;
    float x1 = (float) length1;
    float x2 = (float) length2;

    return sqrt((x1 * x1) + (x2 * x2) - (2 * x1 * x2 * cos(theta_diff * DEG_TO_RAD)));
}

std::vector<int> Navigation::calc_peaks(uint16_t arr[], int size) {
    // Create a vector to store all the peak angles 
    // We use int because the angles are all integers 
    std::vector<int> peaks;

    for (unsigned int i = 1; i < size - 1; i++) {
        // Keep checking the data until the distances stop increasing
        if((arr[i] > arr[i - 1]) && (arr[i] > arr[i + 1])) {
            peaks.push_back(i);
        }
    }

    return peaks;
    
}

int Navigation::calc_min_sweep_angle(float dist) {
    float temp_angle = 2 * atan((ROVER_WIDTH + SAFETY_MARGIN) / (2 * dist)); 
    temp_angle *= DEG_TO_RAD;

    // Now I need to round up - always round up because if the minimum angle is 30.2 degrees, it is safer to have a larger angle 
    // I can do this by adding 1 and then casting to an int, so the decimal places will be 'chopped off' 
    // I also need to ensure that the minimum angle is even so that I have an even number angle either side of... 
    // ... the 'greatest_distance_angle' 
    int min_sweep_angle = (int) (temp_angle + 1); 

    if (min_sweep_angle % 2 != 0) { 
        min_sweep_angle++; 
    } 

    return min_sweep_angle;
}

// Arguments needed:
// - the vector of peak angles 
// - the vector of minimum sweep angles 
// - the entire LiDAR buffer and its size 
std::vector<float> Navigation::calc_gap_width(std::vector<int> peak_angles, std::vector<int> min_sweep_angles, uint16_t buf[], int size) {
    // Define a vector for the gap widths
    std::vector<float> gap_widths;
    
    // Define values 
    int num_checks = 3;
    int lt_count = 0; // 'Less-than count'
    int gt_count = 0; // 'Greater-than count'

    float end_distance1 = 0.0f;
    float end_distance2 = 0.0f;
    int end_angle1 = 0;
    int end_angle2 = 0;
    
    // Define vector of minimum thresholhds
    std::vector<float> dist_thresholds;

    // Calculate the vector of minimum thresholds
    for (int i = 0; i < peak_angles.size(); i++) {
        dist_thresholds.push_back((1 - ALPHA) * buf[peak_angles[i]]);
    }

    for (int j = 0; j < peak_angles.size(); j++) {
        // Left-side scan
        for (int k = peak_angles[j]; k >= peak_angles[j] - (min_sweep_angles[j] / 2); k--) {
            if (buf[k] < dist_thresholds[j]) {
                // Reset lt_count to 0, otherwise it will keep accumulating
                lt_count = 0;

                // Check 'num_checks' times to see if the adjacent points are also below the threshold distance
                for (unsigned int i = 0; i < num_checks; i++) {
                    // Lower clamp
                    if ((k - i) < 0) {
                        break;
                    }

                    // Upper clamp
                    if ((k + i) > size) {
                        break;
                    }

                    if (buf[k - i] < dist_thresholds[j]) {
                        lt_count++;
                    } else {
                        // If we go back above the threshold, then the values are not consistent, so we go back to regular checking
                        break;
                    }
                }

                // If we loop through all the extra 'num_checks' points and all the points are less than the minimum threhsold, we can...
                // ... safely assume that we originally found an edge
                if (lt_count == num_checks) {
                    // Update values
                    // Record the distance measurement at which we cross over, and the angle at which this occurs
                    // Then break out of the for loop
                    // Bear in mind, I need the last good point, not the first bad point. This is why I use j + 1
                    end_distance1 = buf[k + 1];
                    end_angle1 = k + 1;
                    // Now we need to break out of the outer for loop, since the edge has been detected
                    break;
                }
            } else if (buf[k] > dist_thresholds[j]) {
                // We just do nothing because we want the values to be greater than the minimum threshold
                continue;
            }
        }

        // Right-side scan
        for (int k = peak_angles[j]; k <= peak_angles[j] + (min_sweep_angles[j] / 2); k++) {
            if (buf[k] < dist_thresholds[j]) {
                // Reset lt_count to 0, otherwise it will keep accumulating
                gt_count = 0;

                if ((peak_angles[j] + (min_sweep_angles[j] / 2)) >= size) {
                    break;
                }

                // Check 'num_checks' times to see if the adjacent points are also below the threshold distance
                for (unsigned int i = 0; i < num_checks; i++) {
                    if (k + i >= size) {
                        break;
                    }

                    if (buf[k + i] < dist_thresholds[j]) {
                        gt_count++;
                    } else {
                        // If we go back above the threshold, then the values are not consistent, so we go back to regular checking
                        break;
                    }
                }

                // If we loop through all the extra 'num_checks' points and all the points are less than the minimum threhsold, we can...
                // ... safely assume that we originally found an edge
                if (gt_count == num_checks) {
                    // Update values
                    // Record the distance measurement at which we cross over, and the angle at which this occurs
                    // Then break out of the for loop
                    // Bear in mind, I need the last good point, not the first bad point. This is why I use j + 1
                    end_distance2 = buf[k - 1];
                    end_angle2 = k - 1;
                    // Now we need to break out of the outer for loop, since the edge has been detected
                    break;
                }
            } else if (buf[k] > dist_thresholds[j]) {
                // We just do nothing because we want the values to be greater than the minimum threshold
                continue;
            }
        }

        // Ok, we have done both sweeps. Now we calculate the gap width and store it in our vector
        float gap_width = calc_width(end_distance1, end_angle1, end_distance2, end_angle2);
        gap_widths.push_back(gap_width);

        // Ok, now we have finished calculating the gap width
        // Now, reset the variables for the next iteration
        end_distance1 = 0.0f;
        end_distance2 = 0.0f;
        end_angle1 = 0;
        end_angle2 = 0;

    }
    
    return gap_widths;
    
}

int Navigation::choose_direction(std::vector<float> gaps) {
    // Now find the largest gap width
    float largest_gw = 0;
    int index;

    for (int i = 0; i < gaps.size(); i++) {
        if (gaps[i] > largest_gw) {
            largest_gw = gaps[i];
            index = i;
        }
    }

    return index;

}


