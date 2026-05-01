/*
 *         File: matrix.h
 *       Author: Steve Gunn
 *      License: MIT License
 *         Date: 27th October 2018
 *  Description: Data structures and interfaces for operations on matrices.
 */

#ifndef _MATRIX_H
#define _MATRIX_H
#include <vector>
#include <iostream>

/* Data structure */ 

class Matrix {
	public:
        unsigned int rows = 0;
        unsigned int cols = 0;
        std::vector<std::vector<double>> elements;
		Matrix();
		Matrix(const unsigned int rows, const unsigned int cols);
};


#endif