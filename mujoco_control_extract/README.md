# rm_control

## MuJoCo control_extract 运行命令

以下命令默认都在仓库根目录 `/home/shun/MuJoCoBin/rm_control` 运行。
当前 `mujoco_bridge` 默认加载的模型已经是：




```text
mujoco_control_extract/sim/models/wheel_leg_urdf4_self_mesh_all.xml
```

先编译：

```bash
cmake -S mujoco_control_extract/sim -B mujoco_control_extract/build
cmake --build mujoco_control_extract/build --target mujoco_bridge
```

默认会优先查找 `third_party/mujoco`。如果 MuJoCo SDK 放在别的位置，可以这样指定：

```bash
cmake -DMUJOCO_ROOT=/path/to/mujoco -S mujoco_control_extract/sim -B mujoco_control_extract/build
```

推荐 GUI 启动命令：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive stand
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward
```

推荐 headless 验证命令：

```bash
./mujoco_control_extract/build/mujoco_bridge --headless --time 3 --mode safe --drive stand
./mujoco_control_extract/build/mujoco_bridge --headless --time 3 --mode safe --drive forward
```

说明：

```text
推荐优先使用 --mode safe。
--drive stand / --drive forward 是上层运行状态机。
--mode stand 会先走 STAND_UP 起立流程；目前 self_mesh_all.xml 下更稳的是直接从 SAFE 起步。
```

仿真窗口按键：

```text
按住 W：前进
按住 S：后退
松开 W / S：自动回到原地站立 stand
SPACE：暂停 / 继续
ESC：退出
```

默认仿真会锁住车体航向，避免串腿在原地绕圈；如果要调试 yaw 自由度，可以加 `--free-yaw`。

默认前进速度是 `0.20 m/s`，可以用命令行改：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward --forward-speed 0.2
```

如果想启动后直接进入前进状态：

```bash
./mujoco_control_extract/build/mujoco_bridge --mode safe --drive forward
```

只冻结初始姿态，不跑仿真：

```bash
./mujoco_control_extract/build/mujoco_bridge --freeze-init --mode safe --drive stand
```

悬空冻结：

```bash
./mujoco_control_extract/build/mujoco_bridge --freeze-init --hang-init --mode safe --drive stand
```

如果你已经 `cd mujoco_control_extract` 进入子目录，也可以这样运行：

```bash
./build/mujoco_bridge --mode safe --drive stand
./build/mujoco_bridge --mode safe --drive forward
```

push 到 git hub dev 上面的 指令
cd /home/shun/MuJoCoBin/rm_control
git fetch origin
git switch -c dev
git add .gitignore
git commit -m "Ignore Zone.Identifier files"
git push -u origin dev









1byte = 8 bit


#### C语言基础：


结构体：结构体是用来存放一大堆的变量的，这些变量可以是不同类型的，比如一个结构体里面可以存放一个int，一个char，一个float，一个double，甚至可以存放一个结构体。结构体本身可以被看作一个变量类型，这个变量类型被另一个变量所引用之后，我就可以通过这个变量来访问这个结构体里面的变量了。


void 类型就是没有类型，所以一般 void 就是表示一个函数没有返回值。
static 代表的是私有函数，只允许在这个文件之内被调用的函数

void CAN_Send(uint32_t id, uint8_t *data, uint8_t len);
这句话的意思是：
声明一个函数 CAN_Send 
这个函数没有返回值  （void 表示不返回结果）
它需要 3 个参数：
1. id   ：一个 32 位无符号整数 （unsigned integer ，32 bit）
2. data ：一个 uint8_t 类型的数据指针  （指针是一包数据的入口地址）
3. len  ：一个 8 位无符号整数 （unsigned integer ，8 bit）


数组
uint8_t x[8] = {10, 20, 30, 40, 50, 60, 70, 80};
这个意思是：
定义一个变量，
名字叫 data 
它是一个数组 
数组里每个元素都是 uint8_t
数组长度是 8

 
指针：
uint8_t *p;
这个意思是：
定义一个变量，
名字叫 p 
它是一个指针 , 指针指向的一般是一个变量的地址。这里的变量地址按照八位无符号整数进行解释，也就是一个字节。其实就是解读引用的时候用几个字节来解读这个地址。
p+1 指向的是下一个字节的地址。
如果 p 这里的值是 0x1000
那么 p+1 的值就是 0x1001
一般情况下，什么类型的数据就用什么类型的指针。

对于一个数组而言（前面定义的 data 是个数组）

如果我要用指针去存一个数组的地址的话我要这么写：
uint8_t *p = &data ; 

p 存的是 data 这个数组第一个值的地址，也就是 10 的地址
在这里调用的时候 *P 就代表 10




函数指针：
void (*func)(void);
这个意思是：
定义一个变量，
名字叫 func 
它是一个函数指针 
指针指向的函数没有返回值 
指针指向的函数没有参数


地址：
地址是内存的编号：
比如说对于一个地址 0x1000
这里的这个地址对应的就是第4096号格子，在stm32中这个格子的大小是 1byte，也就是八位二进制
如果有一个变量是 uint32_t b ; 
那么这个变量就会连着占四个格子，因为 uint32_t 是 32 bit，也就是四个字节，也就是四个格子




float 4 byte 
double 8 byte

++x 和 x++ 的区别：
++x 是先加1再使用
x++ 是先使用再加1

&& 表示 and
|| 表示 or
! 表示 not

这上面的这三个都是用于进行两边的值的判断，如果满足条件就返回 1，否则返回 0。一般是两边的对应的条件进行判断，比如 a > b && a < c，如果 a 大于 b 并且 a 小于 c，那么就返回 1，否则返回 0。

& 表示 and
| 表示 or
^ 表示 xor
~ 表示 not

这上面的这些符号都是用于进行两边的值的运算，也就是二进制运算，比如 0x01 & 0x02，0x01 和 0x02 的二进制表示都是 0000 0001 和 0000 0010，那么 这个式子按照每一位对应的 and 就是 0000 0000，也就是 0。

左移动和右移动：
<< 表示左移
>> 表示右移

左移就是将二进制数向左移动，右移就是将二进制数向右移动，比如 0x01 << 1，0x01 的二进制表示是 0000 0001，那么左移一位就是 0000 0010，也就是 2。


实际参数和形式参数：
实际参数就是外面自己定义的参数，形式参数就是函数括号里面的参数
传递值的方式是实际参数的值复制一份给形式参数
传递地址的方式是把实际参数的地址传给函数，函数通过地址去修改原来的变量
数组比较特殊，数组只能传递地址，所以说数组被修改的话一定是数组本身的内容会被修改。

c 语言没有 string 所以 说他的 character 实际上是一个字符串数组
char name[] = "Hello";
在内存里，它看起来是这样的： 'H' | 'e' | 'l' | 'l' | 'o' | '\0'
最后一定会有那个‘/0’  因为这个是告诉 c 语言这个字符数组到这里结束了
所以说这个东西是不能写成 char name[5] = " Hello "的 因为他还有一个字符，
要写成 char name[6] = " Hello " (注意这里一定得是双引号，如果是单引号的话要和上面内存地址中的东西一样要拆开来写)


#### mujoco 仿真


urdf 文件：
urdf 文件放在my robot目录底下，传文件的话就直接拷贝进去就行了

目录顺序是 MuJoCoBin 里面的 my robot。

命令行：
simulate ~/MuJoCoBin/urdf13.SLDASM/urdf/urdf13.SLDASM.urdf

cd 可以进入任何文件夹， 直接输入 cd 就可以返回主界面。








#### mat lab 仿真

1. 注意rigid transform 这个命令是有方向的，就是说接的 B 方向和 F 方向存在本质的差别
B = Base frame
F = Follower frame
rigid tansform 的意思是把 F 坐标系相对于 B 平移了多少（注意这里的 F 坐标系是目标坐标系，B 坐标系车体的基本坐标系）

2. 左键是移动现在已经有的线，右键是创建新的分支
3. mat lab 里面的旋转是正方向是符合右手定则的
4. mat lab 的旋转轴的问题，mat lab 的旋转轴里的 x y z 底下接三个数
意思是他是先按照 x 坐标系 旋转这么多，然后再按照 y 坐标系 旋转这么多 最后 按照 z 坐标系旋转这么多。如果说这个 旋转轴是 follower axes 的话，那么 x轴和y轴和z轴 都会随着旋转进行改变。如果是 base axes 的话 参考的就是 b 连接的那个 body 的坐标系。
5. 设置step function 的时候记得 阶跃时间要有一个值，不能是零。 
