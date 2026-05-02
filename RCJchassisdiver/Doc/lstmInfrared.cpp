#include "lstmInfrared.h"


// 加载权重
const double* weights_infrared[] = {
    weight_0_infrared,  // 输入权重
    weight_1_infrared,  // 隐藏权重
    weight_2_infrared,  // 偏置
    weight_3_infrared,  // 输出权重
    weight_4_infrared   // 输出偏置
};

// 初始化隐藏状态和细胞状态
double hidden_state_infrared[hidden_size_infrared] = {0.0f};
double cell_state_infrared[hidden_size_infrared] = {0.0f};
double output_infrared[output_size_infrared];

// 定义激活函数
double tanh_activation_infrared(double x) {
    return tanh(x);
}

// 定义 sigmoid 函数
double sigmoid_infrared(double x) {
    return 1.0 / (1.0 + exp(-x));
}

// LSTM 单步前向传播
void lstm_step_infrared(double input[input_size_infrared]) {
    // 提取权重
    const double* W_i = weights_infrared[0];  // 输入权重 (input_size, 4 * hidden_size)
    const double* W_h = weights_infrared[1];  // 隐藏权重 (hidden_size, 4 * hidden_size)
    const double* bias = weights_infrared[2]; // 偏置 (4 * hidden_size,)
    const double* W_o = weights_infrared[3];  // 输出权重 (hidden_size, output_size)
    const double* output_bias = weights_infrared[4];  // 输出偏置 (output_size,)

    // 临时变量
    double gates[4 * hidden_size_infrared] = {0.0f};

    // 计算输入部分
    for (int i = 0; i < 4 * hidden_size_infrared; i++) {
        for (int j = 0; j < input_size_infrared; j++) {
            gates[i] += input[j] * W_i[j * (4 * hidden_size_infrared) + i];
        }
    }

    // 计算隐藏部分
    for (int i = 0; i < 4 * hidden_size_infrared; i++) {
        for (int j = 0; j < hidden_size_infrared; j++) {
            gates[i] += hidden_state_infrared[j] * W_h[j * (4 * hidden_size_infrared) + i];
        }
        gates[i] += bias[i];  // 加偏置
    }

    // 分割门控信号
    double input_gate[hidden_size_infrared];
    double forget_gate[hidden_size_infrared];
    double cell_gate[hidden_size_infrared];
    double output_gate[hidden_size_infrared];

    for (int i = 0; i < hidden_size_infrared; i++) {
        input_gate[i] = sigmoid_infrared(gates[i]);
        forget_gate[i] = sigmoid_infrared(gates[hidden_size_infrared + i]);
        cell_gate[i] = tanh_activation_infrared(gates[2 * hidden_size_infrared + i]);
        output_gate[i] = sigmoid_infrared(gates[3 * hidden_size_infrared + i]);
    }

    // 更新细胞状态
    for (int i = 0; i < hidden_size_infrared; i++) {
        cell_state_infrared[i] = forget_gate[i] * cell_state_infrared[i] + input_gate[i] * cell_gate[i];
    }

    // 更新隐藏状态
    for (int i = 0; i < hidden_size_infrared; i++) {
        hidden_state_infrared[i] = output_gate[i] * tanh_activation_infrared(cell_state_infrared[i]);
    }

    // 计算输出（添加tanh激活）
    for (int i = 0; i < output_size_infrared; i++) {
        output_infrared[i] = 0.0f;
        for (int j = 0; j < hidden_size_infrared; j++) {
            output_infrared[i] += hidden_state_infrared[j] * W_o[j * output_size_infrared + i];
        }
        output_infrared[i] += output_bias[i];  // 加输出偏置
        output_infrared[i] = tanh_activation_infrared(output_infrared[i]);  // 应用tanh激活函数
    }
}

// void getLocal(double input[input_size]){
//   lstm_step(input);
//   for (int i = 0;i < output_size;i++){
//     Serial.print(output[i]);
//   }
// }


const int direF[8]= {-1,3,2,1,12,11,10,9},direB[8]= {-1,9,8,7,6,5,4,3};
int noBall = 0;
float getBallDire(){
  int ballDireF = maxChannel(&Wire1);
  int strengthF = maxNum(&Wire1);
  int ballDireB = maxChannel(&Wire);
  int strengthB = maxNum(&Wire);
  if(max(strengthF,strengthB) <= 4){
    noBall++;
    if(noBall > 30) return -1;
  }
  else noBall = 0;
//   if(max(strengthF,strengthB) <= 3){
//     return -1;
//   }
  int clock;
  if(strengthF > strengthB){
    clock = direF[ballDireF];
  }
  else{
    clock = direB[ballDireB];
  }
//   Serial.print(clock);
//   Serial.print(" ");
  double input[2] = {sin(clock*(2.0*M_PI/12)),cos(clock*(2.0*M_PI/12))};
  lstm_step_infrared(input);
//   Serial.print(output[0]);
//   Serial.print(" ");
//   Serial.print(output[1]);
//   Serial.print(" ");
  output_infrared[0] = infraFilter[0].update(output_infrared[0]);
  output_infrared[1] = infraFilter[1].update(output_infrared[1]);
  float dire = (atan(output_infrared[0]/output_infrared[1])+(output_infrared[1]<0?M_PI:0))/M_PI*180;
  if(dire < 0) dire += 360;
 // Serial.println(dire);
  //delay(20);
  return dire;
//   double dire = atan(output[0]/output[1]);
//   Serial.println(dire);
}
