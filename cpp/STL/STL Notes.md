## STL Learning

---

### Map (Ordered)

- [x] .at
- [x] .begin
- [x] .cbegin
- [x] .insert

### Unordered Map

- 主要是利用unordered map的查找特性（O(1) 时间）
- 其余基本可以不用了解

### Minor Points

- [x] emplace and push, what is emplace exactly?
  - emplace使用时，不需要进行复制。如果需要复制的话有如下流程：
    1. 临时变量 Constructor
    2. 变量复制 CopyConsructor
    3. 临时变量销毁 deConstructor
  - 以上一套流程比较麻烦 特别是是在构造 / 析构函数比较复杂时
- [x] iterator, finish this after noon. Til 16:00.
  - 迭代器没有什么特别的，就是说：**<u>可以在不需要了解内部实现的情况下，对容器进行迭代</u>**。也就是，不需要size，不需要[]，也不需要知道变量类型，可以提供安全灵活的访问。