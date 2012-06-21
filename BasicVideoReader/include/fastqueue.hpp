#ifndef FASTQUEUE_H
#define FASTQUEUE_H

template <class T>
class FastQueue
{
public:
    FastQueue(int queueSize){
        _buffer = new T[queueSize];
        _front = _back = 0;
        _maxSize = queueSize;
        _size = 0;
    }
    ~FastQueue(){
        delete [] _buffer;
    }

    T &front(){
        return _buffer[_front];
    }

    void pop(){
        if (_size>0){
            _front=(++_front)%_maxSize;
            _size--;
        }else{
            //do nothing
        }
    }

    T &back(){
        return _buffer[_back];
    }

    void push(const T &val){
        if(_size < _maxSize){
            _buffer[_back] = val;
            _back = (++_back)%_maxSize;
            _size++;
        }else{
            //do nothing
        }
    }

    int size(){
        return _size;
    }

    bool empty(){
        return (_size == 0);
    }

private:
    T *_buffer;
    int _front,_back,_maxSize,_size;
};

#endif // FASTQUEUE_H
