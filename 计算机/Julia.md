## 基础操作

启动REPL：`julia`

执行脚本：

```julia
julia script.jl
julia --color=yes script.jl  # 带高亮报错的
```

模式切换：

?：切换到help模式

]：进入pkg模式 在此模式下按？显示相关文档

具体某个命令文档：`?add`

### 安装包

```julia
using Pkg
Pkg.add("Example")  # 安装Example
```



### 基本语法

```julia
println("Hello world!")  # 字符串要用双引号

name = "ikai"
println("Hello $name")
println("1 + 1 = $(1+1)")
\:smile = # 可以输入Emoji
\epsilon # 输入LaTeX数学符号
```

#### 多行注释

```julia
#=
    xxxx
=#
```

#### 标注数据类型

```julia
is_even(x::Int) = x % 2 == 0  # 此时如果输入浮点数就会报错
```

#### 分支

```julia
if cond1
    xxx
elseif cond2
    xxx
else
    xxxx
end
```



#### 数组

和Python类似，索引从1开始，可以用列表推导式`[i for i = 1: 10]`

具名数组类似字典`info = (name="ikai", age=20, wechat="你猜")`

可以广播和MATLAB类似`sin.(A)``foo.(A, B, C)`



#### 复数类型

用im表示虚数单位`1 + 1im`

### 函数声明

```julia
function mysum(A)
    s = 0.0
    for a in A
        s += a
    end
    s  # julia函数会返回函数声明的最后一行
end
```



## 调用Python















# Julia设计模式

## 创建模块和包

```julia
using Pkg
Pkg.add("PkgTemplates")
using PkgTemplates
template = Template(; license = "MIT", user = "tk3369")  # 创建一个Template对象
generate(template, "Calculator")

```





