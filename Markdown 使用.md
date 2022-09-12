# Markdown数学公式使用

## 1. 输入数学公式

### 1. 插入公式

1. 行内插入：`$ 公式内容$`,如：$xyz$,
2. 独立公式：将公式插入到新的行内，并且居中符号`$$公式内容$$`,$$xyz$$

### 2. 上下标

1. 上标*^*如：$x^2$
2. 下标*_*如：$x_2$
3. 组合*{}*如：$x_{13}$
4. 角度`\degree`$120\degree$
5. 向量`\vec{a}`$\vec{a}$
6. 平均值`\overline{a}`$\overline{a}$
7. 下划线`\underline{a}`$\underline{a}$
8. (线性回归，直线方程中的)y尖`\widehat{a}`$\widehat{a}$
9. 颚化符号 等价无穷小`\widetilde{a}`$\widetilde{a}$
10. 一阶导数`\dot{a}`$\dot{a}$
11. 二阶导数`\ddot{a}`$\ddot{a}$

### 3. 汉字，字体与格式

1. ==汉字形式，符号：**\mbox{}**，如：$V_{\mbox{初始}}$，即KaTeX parse error: Expected '}', got '\mbox' at position 4: V_{\̲m̲b̲o̲x̲{初始}}==  $V_{你好}$
2. 字体控制，符号：\displaystyle，加了：、displaystyle字体会变大，如$\displaystyle \frac{x+y}{y+z}$，即 $\displaystyle \frac{x+y}{y+z} $，比 $\frac{x+y}{y+z}$大
3. 下划线，符号：\underline，如：$\underline{x+y}$，即**$\underline{x+y}$ **
4. 标签，符号：\tag{数字}，如：$\tag{11}$，即KaTeX parse error: \tag works only in display equations
5. 上大括号，符号：\overbrace{算式}，如：$\overbrace{a+b+c+d}^{2.0}$，即 **\overbrace{a+b+c+d}^{2.0} **
6. 下大括号，符号：\underbrace{算式}，如：$a+\underbrace{b+c}_{1.0}+d$，即**a+\underbrace{b+c}_{1.0}+d**
7. 上位符号，符号：\stacrel{上位符号}{基位符号}，如：$x\stackrel{\mathrm{def}}{=}{x_1,\dots,x_n}$，即：**$x\stackrel{\mathrm{def}}{=}{x_1,\dots,x_n}**
8. 左大括号：$$ 程序=\left\{ \begin{matrix} 数据结构 \\ 算法  \end{matrix} \right. $$
7. 

### 4. 占位符

1. quad空格，符号：\quad，如：$x \quad y$（注意\quad前后要有空格），即 **x \quad y**
2. 两个quad空格，符号：\qquad，如：$x \qquad y$（注意\qquad前后要有空格），即**x \qquad y**
3. 大空格，符号：**\ ** ，如：$x \ y$，即**x \ y**
4. 中空格，符号：**\: **，如：$x \: y$，即 **x \: y **
5. 小空格，符号：**\, **，如：$x \, y$，即**x \, y**
6. 紧贴，符号：**\\! **，如：$x \! y$，即$x ! y $

### 5. 定界符与组合

|   符号名   |            格式            |                      实例                       |
| :--------: | :------------------------: | :---------------------------------------------: |
|    括号    |       \描述(…\描述)        |                $(), \big(x\big)$                |
|   中括号   |            […]             |                      $[x]$                      |
|   大括号   |            {…}             |                      ${x}$                      |
| 自适应括号 |       \left(\right)        |                $\left(x\right)$                 |
|  组合公式  | {上位公式\choose 下位公式} | ${n+1 \choose k}={n \choose k}+{n \choose k-1}$ |
|  组合公式  | {上位公式 \atop 下位公式}  | ${n+1 \choose k}={n \choose k}+{n \choose k-1}$ |

> 注：括号部分，big还可以换成Big，bigg，Bigg，括号会逐渐增大

### 6. 四则运算

| 符号名 |        格式        |      示例       |
| :----: | :----------------: | :-------------: |
|  加号  |         +          |      $x+y$      |
|  减号  |         -          |     $x - y$     |
|  加减  |        \pm         |    $x \pm y$    |
|  减加  |        \mp         |    $x \mp y$    |
|  乘法  |       \times       |  $x \times y$   |
|  星乘  |        \ast        |   $x \ast y$    |
|  点乘  |       \cdot        |   $x \cdot y$   |
|  除法  |        \div        |   $x \div y $   |
|  斜除  |         /          |      x / y      |
|  分式  | \frac{fenzi}{分母} |  $\frac{x}{y}$  |
|  分式  | {分子} \over{分母} | ${x} \over {y}$ |

**\cdots**是$\cdots$

### 7. 高级运算

|  符号名  |          格式           |                            示例                            |
| :------: | :---------------------: | :--------------------------------------------------------: |
|  平均数  |      \overline{…}       |            $ \overline{x_1, x_2, \cdots, x_n}$             |
| 开二次方 |     \sqrt{被开方数}     |                         $\sqrt{x}$                         |
|   开方   | \sqrt[开方数]{被开方数} |                      $\sqrt[3]{x+y}$                       |
|   对数   |         \log{}          |                        $ \log{x+y}$                        |
|   极限   |         \lim{}          |        $\lim^{x \to \infty}_{y \to 0}{\frac{x}{y}}$        |
|   极限   |  \displaystyle \lim{}   | $\displaystyle \lim^{x \to \infty}_{y \to 0}{\frac{x}{y}}$ |
|   求和   |         \sum{}          |        $\sum^{x \to \infty}_{y \to 0}{\frac{x}{y}}$        |
|          |  \displaystyle \sum{}   | $\displaystyle \sum^{x \to \infty}_{y \to 0}{\frac{x}{y}}$ |
|   积分   |         \int{}          |                  $\int^{\infty}_{0}{xdx}$                  |
|          |  \displaystyle \int{}   |           $\displaystyle \int^{\infty}_{0}{xdx}$           |
|   微分   |       \partial{}        |              $\frac{\partial x}{\partial y}$               |
|          |                         |                                                            |

### 8. 矩阵

`$$\begin{matrix}…\end{matrix}$$`，使用&分隔同行元素，\换行。如：
$$
\begin{matrix}
    1 & x & x^2 \\
    1 & y & y^2 \\
    1 & z & z^2 \\
    \end{matrix}
$$
### 9. 逻辑运算

1. 等于运算，符号：=，如：$x+y=z$，`x + y = z` 
2. 大于运算，符号：>，如：$x+y>z$，`x + y > z`
3. 小于运算，符号：<，如：$x+y<z$，x + y &lt; z x+y&lt;zx+y<z
4. 大于等于运算，符号：\geq，如：$x+y \geq z$，x + y ≥ z x+y \geq zx+y≥z
5. 小于等于运算，符号：\leq，如：$x+y \leq z$，x + y ≤ z x+y \leq zx+y≤z
6. 不等于运算，符号：\neq，如：$x+y \neq z$，x + y ≠ z x+y \neq zx+y ̸=z
7. 不大于等于运算，符号：\ngeq，如：$x+y \ngeq z$，x + y ≱ z x+y \ngeq zx+y≱z
   不大于等于运算，符号：\not\geq，如：$x+y \not\geq z$，x + y ̸ ≥ z x+y \not\geq zx+y 
   ̸≥z
8. 不小于等于运算，符号：\nleq，如：$x+y \nleq z$，x + y ≰ z x+y \nleq zx+y≰z
9. 不小于等于运算，符号：\not\leq，如：$x+y \not\leq z$，x + y ̸ ≤ z x+y \not\leq zx+y 
   ̸≤z
10. 约等于运算，符号：\approx，如：$x+y \approx z$，x + y ≈ z x+y \approx zx+y≈z
11. 恒定等于运算，符号：\equiv，如：$x+y \equiv z$，x + y ≡ z x+y \equiv zx+y≡z



## 2. 其他常用符号

$\nabla$ : `\nabla`

$\alpha$ : `\alpha`

$\beta$  : `\beta`

$\psi$ : `\psi`

$\phi$ : `\phi`

其他参考

![v2-86e0ce0769420aee8fcc1e020d86f6e1_b](https://s2.loli.net/2022/08/03/VRpwMfTL1qrWelx.png)

> 把首字母改为大写即可变成大写如$\Psi$



|     中文     |                     代码                      |                     效果                      |
| :----------: | :-------------------------------------------: | :-------------------------------------------: |
|     无穷     |                   `\infty`                    |                   $\infty$                    |
|     空集     |          `\emptyset`或`\varnothing`           |          $\emptyset$或$\varnothing$           |
|     属于     |                     `\in`                     |                     $\in$                     |
|    不属于    |                   `\notin`                    |                   $\notin$                    |
|    不等于    |                `\neq`或`\not=`                |                 $\neq或\not=$                 |
|   大于等于   |              `\geq`或`\geqslant`              |               $\geq或\geqslant$               |
|   小于等于   |              `\leq`或`\leqslant`              |               $\leq或\leqslant$               |
|    包含于    |                  `\subseteq`                  |                  $\subseteq$                  |
|   真包含于   |                 `\subsetneqq`                 |                 $\subsetneqq$                 |
|     包含     |                  `\supseteq`                  |                  $\supseteq$                  |
|    真包含    |                 `\supsetneqq`                 |                 $\supsetneqq$                 |
|     交集     |                    `\cap`                     |                    $\cap$                     |
|     并集     |                    `\cup`                     |                    $\cup$                     |
|      非      |                    `\neg`                     |                    $\neg$                     |
|      或      |                    `\lor`                     |                    $\lor$                     |
|      与      |                    `\land`                    |                    $\land$                    |
| 至少存在一个 |                   `\exists`                   |                   $\exists$                   |
|  只存在一个  |                  `\exists!`                   |                  $\exists!$                   |
|    对全部    |                   `\forall`                   |                   $\forall$                   |
|     推出     |       `\Rightarrow`或`\Longrightarrow`        |        $\Rightarrow或\Longrightarrow$         |
|    被推出    |        `\Leftarrow`或`\Longleftarrow`         |         $\Leftarrow或\Longleftarrow$          |
|    等价于    |           `\Leftrightarrow`或`\iff`           |            $\Leftrightarrow或\iff$            |
|     向量     |               `\overrightarrow`               |             $\overrightarrow{a}$              |
|     垂直     |                    `\perp`                    |                    $\perp$                    |
|    不垂直    |                  `\not\pert`                  |                  $\not\perp$                  |
|     平行     |                  `\parallel`                  |                  $\parallel$                  |
|    不平行    |                 `\nparallel`                  |                 $\nparallel$                  |
|     相似     |                    `\sim`                     |                    $\sim$                     |
|     全等     |                    `\cong`                    |                    $\cong$                    |
|    三角形    |                  `\triangle`                  |                  $\triangle$                  |
|     角度     |                   `\angle`                    |                   $\angle$                    |
|     加减     |                     `\pm`                     |                     $\pm$                     |
|     减加     |                     `\mp`                     |                     $\mp$                     |
|     乘法     |                   `\times`                    |                   $\times$                    |
|     除法     |                    `\div`                     |                   $$\div$$                    |
|     向量     |           `\vec`或`\overrightarrow`           |         $\vec a或\overrightarrow {a}$         |
|     点乘     |                    `\cdot`                    |                    $\cdot$                    |
|    省略号    |              `\cdots`或`\ldots`               |               $\cdots或\ldots$                |
|     分数     |           `\frac{}{}`或`\dfrac{}{}`           |          $\frac{a}{b}或\dfrac{a}{b}$          |
|     开方     |                  `\sqrt[]{}`                  |                 $\sqrt[3]{a}$                 |
|     积分     |                `\int_0^1x^2dx`                |                $\int_0^1x^2dx$                |
|     极限     | `\lim\limits_{x\rightarrow\infty}\frac{1}{x}` | $\lim\limits_{x\rightarrow\infty}\frac{1}{x}$ |
|     累加     |            `\sum_1^n\frac{1}{x^2}`            |            $\sum_1^n\frac{1}{x^2}$            |
|     累乘     |        `\prod_{i=0}^n{1 \over {x^2}}`         |        $\prod_{i=0}^n{1 \over {x^2}}$         |
|     对数     |             `\log`或`\ln`或`\lg`              |            $\log_3 3或\ln3或\lg3$             |
|   三角函数   |    `\sin`或`\tan`或`\cot`或`\sec`或`\csc`     |            $\sin \alpha\sec \beta$            |
|     因为     |                  `\because`                   |                  $\because$                   |
|     所以     |                 `\therefore`                  |                 $\therefore$                  |
|    上划线    |             `\overline{a+b+c+d}`              |             $\overline{a+b+b+d}$              |
|    下划线    |               `\underline{a+b}`               |               $\underline{a+b}$               |
|    上括号    |            `\overbrace{a+b}^{3个}`            |            $\overbrace{a+b}^{3个}$            |
|    下括号    |           `\underbrace{a+b}_{3个}`            |           $\underbrace{a+b}_{3个}$            |
|     连线     |      `\hat{y}`或`\check{y}`或`\breve{y}`      |        $\hat{y}或\check{y}或\breve{y}$        |
|   转义符号   |            `\# \$ \% \& \_ \{ \}`             |               $\#\$\%\&\_\{\}$                |
|     空格     |                    `    `                     |                                               |
|    相当于    |                   `\equiv`                    |                   $\equiv$                    |
