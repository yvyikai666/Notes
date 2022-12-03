# LaTeX

## LaTeX源码结构

```latex
\documentclass{...}  % ...为某文档
% 导言区
\begin{document}
% 正文内容
\end{document}
% 此后的内容被忽略
```

## 插入其他文件

```latex
\include{<filename>}  % 不包括.tex,可以使用相对路径或绝对路径，读入之前会另起一页
\input{<filename>}  % 不会另起一页，复杂的导言，图表和代码常用
```

## 只检查错误不生成PDF

~~~tex
\usepackage{syntonly}
\syntaxonly  % 不用的话就注释掉
~~~



