# simulink学习

## matlab基础

分脚本编程，函数编程，面向对象编程

matlab函数名必须和文件名一致：

例：

```matlab
function [output1, output2] = mlex2(input1, input2)
output1 = input1
output2 = local_fun(input)
end
function [result] = local_fun(input)
    result = input * 2
end
```

面向对象时类名也要和文件名一样其他和python差不多（但不太常用）

## simulink基础

仿真的时候最好用变步长，速度和精度都比较好

CTRL+用鼠标拖动就可以复制模块

ctrl+点一下可以自动连线 

模型引用使用Unspecified Model Name模块

使用加速模式回变成mex文件运行更快
