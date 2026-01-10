#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

/**
 * @brief 执行SVD分解的封装函数
 * 
 * @param inputSrc 输入矩阵 (必须是 CV_32F 或 CV_64F)
 * @param U        输出左奇异向量矩阵
 * @param W        输出奇异值 (注意：OpenCV默认返回的是列向量，不是对角矩阵)
 * @param Vt       输出右奇异向量的转置 (V^T)
 */
void performSVD(const Mat& inputSrc, Mat& U, Mat& W, Mat& Vt) {
    // 检查输入矩阵是否为空
    if (inputSrc.empty()) {
        cerr << "Error: Input matrix is empty!" << endl;
        return;
    }

    // 确保输入是浮点型，SVD不支持整数型矩阵
    Mat srcFloat;
    if (inputSrc.type() != CV_32F && inputSrc.type() != CV_64F) {
        cout << "Converting input to CV_64F..." << endl;
        inputSrc.convertTo(srcFloat, CV_64F);
    } else {
        srcFloat = inputSrc;
    }

    // 调用OpenCV的SVD计算
    // 参数说明：
    // srcFloat: 输入矩阵
    // W: 计算出的奇异值向量
    // U: 计算出的左奇异向量
    // Vt: 计算出的右奇异向量的转置
    // flags: 可选参数，通常默认即可。
    //        SVD::FULL_UV (计算完整的U和Vt) 
    //        SVD::MODIFY_A (允许修改原矩阵以加快速度)
    cv::SVD::compute(srcFloat, W, U, Vt);
}

int main() {
    // 1. 准备数据
    // 创建一个 3x4 的示例矩阵
    double data[3][4] = {
        { 10, 20, 30, 40 },
        { 5,  1,  2,  3  },
        { 9,  7,  5,  1  }
    };
    Mat A = Mat(3, 4, CV_64F, data).clone(); // 拷贝数据确保内存安全

    cout << "Original Matrix A:" << A << endl;

    // 2. 定义用于存储结果的矩阵
    Mat U, W, Vt;

    // 3. 执行 SVD
    performSVD(A, U, W, Vt);

    // 4. 输出结果
    cout << "SVD Results:" << endl;
    cout << "-------------------------" << endl;
    cout << "W (Singular Values):" << W << endl; // 注意：这是一个列向量
    cout << "U (Left Singular Vectors):" << U << endl;
    cout << "Vt (Right Singular Vectors Transpose):" << Vt << endl;
    cout << "-------------------------" << endl;

    // 5. 验证结果 (重构矩阵 A = U * diag(W) * Vt)
    // OpenCV计算出的W是向量，乘法前需要转为对角矩阵
    Mat W_diag = Mat::diag(W); 
    
    // 矩阵乘法
    Mat reconstructed = U * W_diag * Vt;

    cout << "Reconstructed Matrix (U * W_diag * Vt):" << reconstructed << endl;

    // 6. 检查误差
    Mat diff;
    absdiff(A, reconstructed, diff);
    double error = sum(diff)[0]; // 求所有元素差值的和
    
    cout << "Reconstruction Error (L1 Norm): " << error << endl;

    return 0;
}
