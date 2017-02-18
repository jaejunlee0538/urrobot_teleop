//
// Created by ub1404 on 17. 2. 18.
//
#include <iostream>
#include "BytesBuffer.h"
#include <stdint.h>
template <typename T>
void test(BytesBuffer& buffer, T v_des){
  T tmp;
  buffer.unload(tmp);
  if(tmp != v_des)
    std::cerr<<"Fail"<<std::endl;
  else
    std::cerr<<"Pass"<<std::endl;
}

int main(){
  BytesBuffer buffer;

  int32_t val1 = 50;
  int32_t val2 = 60;
  int16_t  val3 = 70;
  float val4 = 1.315f;
  double val5 = 3.152;
  bool val6 = true;
  int8_t val7 = 5;

  buffer.load(val1);
  buffer.load(val2);
  buffer.load(val3);
  buffer.load(val4);
  buffer.load(val5);
  buffer.load(val6);
  buffer.load(val7);

  test(buffer, val1);
  test(buffer, val2);
  test(buffer, val3);
  test(buffer, val4);
  test(buffer, val5);
  test(buffer, val6);
  test(buffer, val7);
  return 0;
}