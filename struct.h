
#ifndef STRUCT_H
#define STRUCT_H

#include <stdint.h>
#include <queue>
#include <mutex>
#include <iostream>

#include "mavlink2/common/mavlink.h"


template <class T>
class CircularBuffer{
	std::vector <T> data;
	int maxLen;
	unsigned int n;
	int f,b;
	std::mutex mu;

	void cntInc(int &i){
		i++;
		if(i==maxLen)
			i=0;
	}

	void cntDec(int &i){
		i--;
		if(i<0)
			i=maxLen;
	}

public:
	CircularBuffer(int l){
		if(l>1)
			maxLen=l;
		else
			maxLen=0;
		data.resize(maxLen);
		n=0;f=0;b=0;
	}

	CircularBuffer(){
		maxLen=1;
		data.resize(maxLen);
		n=0;f=0;b=0;
	}

	void display(){
		std::cout<<"\nSize: "<<size()<<", "<<b<<","<<f<<"\n";
		for(int i=b,cnt=0;cnt<n;cntInc(i),cnt++){
			std::cout<<data[i]<<" ";
		}
	}
	bool push(T ele){
		mu.lock();

		if(n==0){
			data[f]=ele;
			cntInc(f);
			n++;
			mu.unlock();
			return 1;
		}

		else if(f!=b){
			data[f]=ele;
			cntInc(f);
			n++;
			mu.unlock();
			return 1;
		}
		else{
			cntInc(b);
			data[f]=ele;
			cntInc(f);
			mu.unlock();
			return 0;
		}

	}

	bool push(T *ele,int l){
		bool safe=true;
		for(int i=0;i<l;i++){
			if(!push(ele[i])){
				safe=false;
			}
		}
		return safe;
	}

	bool pop(T &ele){
		if(n){
			mu.lock();
			ele=data[b];
			if(n>1){
				cntInc(b);
			}
			n--;
			if(n==0){
				f=b;
			}
			mu.unlock();
			return 1;
		}
		return 0;
	}

	bool copy(T &ele){
		if(n){
			mu.lock();
			ele=data[b];
			mu.unlock();
			return 1;
		}
		return 0;
	}

	bool pop(T *ele,int l){
		bool safe=true;
		for(int i=0;i<l;i++){
			if(!pop(ele[i])){
				safe=false;
				break;
			}
		}
		return safe;
	}

	size_t size(){
		return n;
	}

	void clear(){
		data.clear();
		n=0;f=0;b=0;
	}
};

#endif
