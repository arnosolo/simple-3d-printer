#ifndef _ARRAY_H_
#define _ARRAY_H_

template<class T>
class Array {
  typedef void func_t(T, int);
  private:
    T *_startAddress;
    int _length = 0;
    int _capacity;
  public:
    Array(int capacity = 5) {
      _capacity = capacity;
      _startAddress = new T[capacity];
    }

    ~Array() {
      delete[] _startAddress;
    }

    void clean() {
      _length = 0;
      delete[] _startAddress;
      _startAddress = new T[_capacity];
    }

    /**
    * @brief  Overload '[]', so we can access 5th item of array like arr[5]
    * @param  index index of array
    * @return demanded item or T() if index out of range
    */
    T operator[] (int index) {
      if(index >= _length || index < 0) {
        // Serial.println("index out of range of array"); // I not sure if I should print this msg
        return T();
      } else {
        return *(_startAddress + index);
      }
    };

    /**
    * @brief  Overload '=', so we can copy array like arrA = arrB
    * @param  anotherArr another array
    * @return
    */
    Array & operator= (const Array &anotherArr) {
      // If anotherArr has nothing
      if (anotherArr._length == 0) {
        if (_startAddress) delete[] _startAddress;
        _startAddress = NULL;
        _length = 0;
        return *this;
      }

      // If the original array is smaller than new array
      if (_length < anotherArr._length) {
        if (_startAddress) delete[] _startAddress;
        _startAddress = new T[anotherArr._length];
      }

      memcpy(_startAddress, anotherArr._startAddress, sizeof(T) * anotherArr._length);
      _length = anotherArr._length;
      return *this;
    };

    /**
    * @brief  Add an item at the end of array
    * @param  item item needs be added
    * @return
    */
    void push(const T &item) {
      if(_length == _capacity) {
        _capacity += 5;
        T *newAddress = new T[_capacity];
        memcpy(newAddress, _startAddress, _length * sizeof(T));
        _startAddress = newAddress;
      }
      *(_startAddress + _length) = item; // write item into memory
      _length ++;
    };
    
    /**
    * @brief  Remove last item from array, this operation will change the array
    * @param
    * @return Last item of the array, or a T() if nothing is in array
    */
    T pop() {
      if(_length == 0) {
        return T();
      } else {
        _length --;
        return *(_startAddress + _length);
      }
    };

    /**
    * @brief  iterate over all items in an array
    * @param  callback should looks like printStr(String str) if your array looks like Array<String> arr;
    * @return 
    */
    void forEach(func_t callback) {
      for(int i=0; i < _length; i++) {
        callback(*(_startAddress + i), i);
      }
    }

    int length() { return _length; };
};

#endif 