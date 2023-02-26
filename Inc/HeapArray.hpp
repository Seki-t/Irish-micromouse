#pragma once
#include <stdint.h>
#include <array>
// 最小値が先頭のヒープに要素を追加

#define HEAP_MAX_SIZE 300

template <typename T>
class HeapArray
{
  std::array<T, HEAP_MAX_SIZE> arr;
  int size;

public:
  HeapArray() : size(0)
  {
  }

  void push(T element)
  {
    int last = size;
    arr[last] = element;
    size += 1;

    while (last > 0)
    {
      int i = (last - 1) / 2;
      // 親と値を入れ替え
      if (element >= arr[i])
        break;

      arr[last] = arr[i];
      last = i;
    }
    arr[last] = element;
  }

  T pop()
  {

    T popped = arr[0];
    T last_value = arr[size - 1];
    size -= 1;

    int i = 0;
    while ((2 * i + 1) < size)
    {

      int left = 2 * i + 1;
      int right = left + 1;

      if (right < size && arr[right] < arr[left])
        left = right;

      if (arr[left] >= last_value)
        break;
      arr[i] = arr[left];
      i = left;
    }
    arr[i] = last_value;

    return popped;
  }

  int getSize(){return size;}

  void reset(){
    size = 0;
  }
};
