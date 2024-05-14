#include "matrixOperations.h"
#include <iostream>
#include <string>
#include <cmath>
using namespace std;

int main2() {
    // Example usage
    vec userConfig = { 1.0, 2.0, 3.0, 45.0 };  // (x, y, z, phi)
    matrix internalMatrix;

    // Convert User input TO Internal representation
    UTOI(userConfig, internalMatrix);

    // Print the resulting matrix
    std::cout << "Internal Representation Matrix (Transformation Matrix T):\n";
    printMatrix(internalMatrix);

    // Convert it back to user form
    vec userFormResult;
    ITOU(internalMatrix, userFormResult);

    /*matrix multOp;
    TMULT(internalMatrix, internalMatrix, multOp);
    std::cout << "\nTwo matrixes multiplied to each other:\n";
    printMatrix(multOp);*/

    matrix brela = {
        {1.0, 2.0, 3.0, 4.0},
        {5.0, 6.0, 7.0, 8.0},
        {9.0, 10.0, 11.0, 12.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    cout << endl;
    printMatrix(brela);

    matrix crelb = {
        {2.0, 0.0, 0.0, 1.0},
        {0.0, 2.0, 0.0, 2.0},
        {0.0, 0.0, 2.0, 3.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    cout << endl;
    printMatrix(crelb);

    matrix resultMatrix;

    // Perform matrix multiplication
    cout << endl; 
    TMULT(brela, crelb, resultMatrix);
    printMatrix(resultMatrix);

    cout << endl;
    TMULT(internalMatrix, internalMatrix, resultMatrix);
    printMatrix(resultMatrix);

    matrix invertMatrix; 
    cout << endl;
    TINVERT(resultMatrix, invertMatrix);
    printMatrix(invertMatrix);


    // Print the resulting vector
    std::cout << "\nUser Form Vector after ITOU:\n";
    for (int i = 0; i < MVSIZE; i++) {
        std::cout << userFormResult[i] << " ";
    }

    std::cout << " TESTING INVKIN: " << endl; 

    return 0;
}