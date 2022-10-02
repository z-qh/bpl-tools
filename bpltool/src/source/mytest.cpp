#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>

class CThreadDemo {
private:
 std::deque<int> m_data;
 std::mutex mtx;
 std::condition_variable m_cv;
 int m_nGen;

private:
 void productThread() {
  while (true) {
   std::unique_lock<std::mutex> lck(mtx);
   m_nGen = ++m_nGen % 1000;
   printf("product :%d\n", m_nGen);
   //std::cout << "product : " << m_nGen << std::endl;
   m_data.push_back(m_nGen);
   lck.unlock();
   m_cv.notify_all();

   std::chrono::seconds dura(1);
   std::this_thread::sleep_for(dura);
  }
 }

 void consumeThread() {
  while (true) {
   std::unique_lock<std::mutex> lck(mtx);
   while (m_data.empty()) {
    m_cv.wait(lck);
   }
   int data = m_data.front();
   m_data.pop_front();
   printf("consume %d\n", data);
   lck.unlock();
   
   std::chrono::seconds dura(2);
   std::this_thread::sleep_for(dura);
  }
 }

public:
 CThreadDemo() {
  m_data.clear();
  m_nGen = 0;
 }

 void run() {
  std::vector<std::thread> threads;
  threads.clear();
  for (int i = 0; i < 1; ++i) {
   threads.emplace_back(std::thread(&CThreadDemo::productThread, this));
  }
  for (int i = 0; i < 1; ++i) {
   threads.emplace_back(std::thread(&CThreadDemo::consumeThread, this));
  }
  for (auto& t : threads) {
   t.join();
  }
 }
};

int main(){
    CThreadDemo A;
    A.run();
}

