//
// Created by liyubo on 12/15/17.
//

#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include"hw.h"
#define MATRIX_SIZE 50

void homework()
{


//作业
    Eigen::MatrixXd matrix_A;
    matrix_A = Eigen::MatrixXd::Random( 100, 100 );
    Eigen::MatrixXd matrix_b;
    matrix_b = Eigen::MatrixXd::Random( 100, 1 );
    Eigen::MatrixXd x;
// cout << matrix_A << endl;
// cout << matrix_b << endl;
    x = matrix_A.llt().solve(matrix_b);   //llt Cholesky来解方程


    /*******************时间比较*********************/

    clock_t time_stt = clock();
    x = matrix_A.colPivHouseholderQr().solve(matrix_b);  //利用QR分解求解方程
    cout <<"time use in Qr decomposition is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;


    time_stt = clock();
    x = matrix_A.fullPivLu().solve(matrix_b);   //LU
    cout <<"time use  in fullPivLu  is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;


    time_stt = clock();
    x = matrix_A.inverse()*matrix_b;;  //利用求逆来解方程
    cout <<"time use  in normal inverse  is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;


    time_stt = clock();
    x = matrix_A.llt().solve(matrix_b);   //llt Cholesky来解方程
    cout <<"time use  in llt(Cholesky)  is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    time_stt = clock();
    x = matrix_A.ldlt().solve(matrix_b);   //ldlt
    cout <<"time use  in ldlt is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms"<< endl;
}
