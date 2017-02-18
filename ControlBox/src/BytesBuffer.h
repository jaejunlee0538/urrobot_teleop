//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_BYTESBUFFER_H
#define PLAYGROUND_BYTESBUFFER_H
#include <deque>
#include <stdint.h>
#include <stdlib.h>
namespace BytesBufferHelper {
  template<typename T>
  inline void SET_BIT(T &val, int flag_offset) {
    val |= ((T) 1) << flag_offset;
  }

  template<typename T>
  inline void CLEAR_BIT(T &val, int flag_offset) {
    val &= ~(((T) 1) << flag_offset);
  }

  template<typename T>
  inline void CLEAR_ALL_BIT(T &val) {
    val = (T) 0;
  }

  template<typename T>
  inline bool IS_BIT_SET(T &val, int flag_offset) {
    return (bool) (val & (((T) 1) << flag_offset));
  }
}

#define LOAD_AND_RETURN_IF_FAILED(buffer, val)  if(!buffer.load(val)) return false;
#define UNLOAD_AND_RETURN_IF_FAILED(buffer, val) if(!buffer.unload(val)) return false;
class BytesBuffer
{
public:
  BytesBuffer(void){}

  ~BytesBuffer(void){}

  void clear(){
    _buffer.clear();
  }

  /*
   * make sure output array is allocated enough memory
   */
  void copyToArray(char* output){
    std::copy(_buffer.begin(), _buffer.end(), output);
  }

  /*
   * initialize BytesBuffer with data of len bytes.
   */
  bool init(const char* data, const size_t& len){
    if(len > maxSize()){
      return false;
    }
    _buffer.clear();
    _buffer.insert(_buffer.end(), data, data+len);
    return true;
  }

  bool load(const bool& val){
    int8_t tmp = val;
    return load(&tmp, 1);
  }
  bool load(const uint8_t& val){
    return load(&val, 1);
  }
  bool load(const int8_t& val){
    return load(&val, 1);
  }
  bool load(const uint16_t& val){
    return load(&val, 2);
  }
  bool load(const int16_t& val){
    return load(&val, 2);
  }
  bool load(const uint32_t& val){
    return load(&val, 4);
  }
  bool load(const int32_t& val){
    return load(&val, 4);
  }
  bool load(const uint64_t& val){
    return load(&val, 8);
  }
  bool load(const int64_t& val){
    return load(&val, 8);
  }
  bool load(const float& val){
    return load(&val, sizeof(float));
  }
  bool load(const double& val){
    return load(&val, sizeof(double));
  }
  bool load(const void* val, const size_t& len){
    if(size()+len > maxSize()){
      return false;
    }
    //must not use memcpy to store an array
    const char* p_val = (const char*)val;
    _buffer.insert(_buffer.end(), p_val, p_val+len);
    return true;
  }

  bool unload(bool& val){
    int8_t tmp;
    if(!unload(&tmp, 1)) {
      return false;
    }
    val = static_cast<bool>(tmp);
    return true;
  }
  bool unload(uint8_t& val){
    return unload(&val, 1);
  }
  bool unload(int8_t& val){
    return unload(&val, 1);
  }
  bool unload(uint16_t& val){
    return unload(&val, 2);
  }
  bool unload(int16_t& val){
    return unload(&val, 2);
  }
  bool unload(uint32_t& val){
    return unload(&val, 4);
  }
  bool unload(int32_t& val){
    return unload(&val, 4);
  }
  bool unload(uint64_t& val){
    return unload(&val, 8);
  }
  bool unload(int64_t& val){
    return unload(&val, 8);
  }
  bool unload(float& val){
    return unload(&val, sizeof(float));
  }
  bool unload(double& val){
    return unload(&val, sizeof(double));
  }
  bool unload(void* val, const size_t& len){
    if(len > size()){
      //in-buffer data is not sufficient to unload len bytes
      return false;
    }
    //must not use memcpy to extract an array data
    std::copy(_buffer.begin(), _buffer.begin()+len, (char*)val);
    _buffer.erase(_buffer.begin(), _buffer.begin()+len);
    /*
    std::deque<char>::const_iterator iter = _buffer.begin();
    std::deque<char>::const_iterator end = _buffer.begin()+len;
    while(iter != end){
      *val = *iter;
      iter++;
      val++;
    }
    _buffer.erase(_buffer.begin(), end);
     */
    return true;
  }

  size_t size() const{
    return _buffer.size();
  }

  size_t maxSize() const{
    return _buffer.max_size();
  }
private:
  std::deque<char> _buffer;
};


#endif //PLAYGROUND_BYTESBUFFER_H
