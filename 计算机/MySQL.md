# MySQL

## 1.介绍

Mysql是一个关系型数据库管理系统软件

服务端软件 Server

客户端软件 Client



RDMBMS是关系型数据库管理软件系统是一个软件

Mysql是RDBMS中的一个





SQL：数据库的语言（structured query language 结构化查询语言)

    用来操作PDBMS



- 语言的分类

|DQL|数据查询语言|select, |
|---|---|---|
|DML|数据操作|insert, update, delete|
|DDL|定义|create, drop|
|TPL|事务处理| |
|DCL|数据控制| |







## 2.MYSQL数据类型

|int, bit|整数|
|---|---|
|decimal|小数|
|varchar，char|字符串（varchar是可变字符串）|
|date，time，datetime|日期，时间|
|enum|枚举类型|





## 3.Mysql字段约束

|primary key|主键约束|
|---|---|
|not null|非空约束|
|unique|唯一约束|
|default|默认约束|





## 4.Mysql的安装（Linux）

服务端

```Bash
sudo apt-get install mysql-server
```


客户端

```Bash
sudo apt-get install mysql-client
```




## 5.Mysql server 的启动

- 查看MySQL的状态

```Bash
sudo service mysql status

```


- 停止MySQL服务

```Bash
sudo service mysql stop
```


- 启动MySQL服务

```Bash
sudo service mysql start

```


- 重启

```Bash
sudo service mysql restart
```






## 6.MySQL配置文件

- 1.port端口默认是3306
- 2.bind-address 服务器绑定ip默认127.0.0.1
- 3.datadir数据库保存路径为/var/lib/mysql
- 4.log-error表示错误日志默认 /var/log/mysql/error.log





## 7.navicat介绍和安装

- 解压操作

```Bash
tar -zxvf ....tar.gz
```


- 查看用户名和密码

```Bash
sudo cat /etc/mysql/debian.cnf
```


用navicat操作表和excel一样





## 8.MySQL终端指令操作

- 1.连接MySQL

```Bash
mysql -uroot -p
```


- 2.显示当前时间

```Bash
select show()

```


- 3.退出连接

```Bash
exit / quit / ctrl+d
```






## 9.MySQL数据库操作

- 1.查看

```SQL
show datadases;
```


- 2.创建

```SQL
create database 数据库名 charset=set;
```


- 3.使用

```SQL
use 数据库名;
```


- 4.查看当前使用的数据库

```SQL
select database;
```


- 5.删除

```SQL
drop databases 数据库名
```






## 10.MySQL表操作

- 1.查看当前库中所有表

```SQL
show tables;

```


- 2.创建

```SQL
create table 表名;
```


- 3.修改表字段类型

```SQL
alter table 表名 modify 列名类型约束;
```


- 4.删表

```SQL
drop table 表名;
```


- 查看表的结构

```SQL
desc 表名;
```






## 11.MySQL-CRUD操作

- 1.查询数据

```SQL
select * from 表名;
```


- 2.查询指定列

```SQL
select id, name from student;
```






## 12.使用Python交互MySQL数据库

### 1.pymysql的使用

- 1.导包

```Python
import pymysql

```


- 2.创建MySQL服务端连接对象

```Python
pymysql.connect(参数列表）
```


- 3.获取游标对象

```Python
carsor = conn.cursor()
```


- 4.执行sql语句

```Python
row_count = cursor.execute(sql)
```


- 5.获取查询结果

```Python
result = cursor.fetchall()
```


- 6.将增加和修改操作提交到数据库

```Python
conn.commit()
```


- 7.回滚数据

```Python
conn.rollback()
```


- 8.关闭游标对象

```Python
cursor.close
```


- 9.关闭连接

```Python
conn.close()
```




### 2.编程分为8步

- 1.导包
- 2.连接MySQL服务
- 3.创建游标对象
- 4.编写sql语句
- 5.使用游标对象调用sql
- 6.获取查询结果
- 7.关闭游标对象
- 8.关闭连接

### 3.例

```Python
import pymysql

con = pymysql.Connect(host='', 
                      user='',
                      password='',
                      database='',
                      port='', 
                      charset='utf8'
                      )
          
cur = con.cursor()
sql = 'select * from students;'
cur.execute(sql)  
result = cur.fetchall()
print(result)
cur.close()
con.close()                   

```












































