/*
 *         File: matrix.c
 *       Author: Steve Gunn
 *      License: MIT License
 *         Date: 27th October 2018
 *  Description: Implementation to dynamically allocate and perform operations on matrices.
 */

#include <stdio.h>
#include <stdlib.h>
#include "matrix.hpp"
#include <vector>
#include <iostream>


// Default constuctor
Matrix::Matrix() {}

Matrix::Matrix(const unsigned int rows, const unsigned int cols): rows(rows), cols(cols) {
	elements.resize(rows, std::vector<double>(cols));
}
	
