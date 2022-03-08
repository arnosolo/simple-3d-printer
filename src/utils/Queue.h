#ifndef _QUEUE_H_
#define _QUEUE_H_

template<class T>
class Queue {
  private:
  public:
    T *_data;
    int _firstIndex;
    int _lastIndex;
    int _length;
    int _capacity;
    Queue(int capacity) {
      _data = new T[capacity];
      _capacity = capacity;
      _firstIndex = 0;
      _lastIndex = capacity -1;
      _length = 0;
    }

    ~Queue() {
      delete[] _data;  
    }

    int length();
    bool isEmpty();
    bool isFull();
    T first();
    T last();
    bool enqueue(const T &item);
    T dequeue();
};

template <class T>
bool Queue<T>::enqueue(const T &item)
{
  if (isFull())
  {
    return false;
  } else {
    _lastIndex = (_lastIndex + 1) % _capacity;
    _data[_lastIndex] = item;
    _length ++;
    return true;
  }
}

template <class T>
T Queue<T>::dequeue()
{
  if (isEmpty())
  {
    return T(); // return empty
  } else {
    int prev = _firstIndex;
    _firstIndex = (_firstIndex + 1) % _capacity;
    _length --;
    return _data[prev];
  }
}

template <class T>
T Queue<T>::first()
{
  if (isEmpty())
  {
    return T(); // return empty
  } else {
    return _data[_firstIndex];
  }
}

template <class T>
T Queue<T>::last()
{
  if (isEmpty())
  {
    return T(); // return empty
  } else {
    return _data[_lastIndex];
  }
}

template <class T>
bool Queue<T>::isEmpty() {
  return ((_length == 0) && ((_lastIndex+1)%_capacity == _firstIndex));
  // return ((_lastIndex+1)%_capacity == _firstIndex);
}

template <class T>
bool Queue<T>::isFull() {
  return ((_length >= _capacity) && ((_lastIndex+1)%_capacity == _firstIndex));
  // return ((_lastIndex+1)%_capacity == _firstIndex);
}

template <class T>
int Queue<T>::length() {
  return _length;
}

#endif 