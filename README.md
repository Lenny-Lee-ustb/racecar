# fork来的库，现在车上跑的就是这个工程，以后以后改完代码觉得没问题就传上来吧～～
# 该版本时间2020.3.26 

待解问题：
1. 车与规划终点的距离始终大于1米（不知在何处更改这个值）
2. teb暂时无法出现倒车的情况
3. 编码器未添加，需要提前做好相关代码的实现（上位机与下位机均还未开始），编码器同时可以拿来作为速度闭环
4. 目前controller订阅的规划为全局规划，teb并没有被controller使用。（是个大问题！！）
5. pf的定位的tf转换还未完成

一些参考网站：
1. ROS中的这些局部导航算法你用过哪些 https://www.guyuehome.com/5500

