# path_planning: Frenet下的无人车路径规划



无人车路径搜索，是一个比较复杂的问题，但是要入门无人车路径规划或者路径搜索，需要先掌握一些基础的东西。不同于机器人中经常看到的路径规划(比如ROS中基于costmap)算法，无人车的路径搜索算法更加先进，它将考虑的东西更多，那么我们先从最简单的开始。

很多时候网上的所谓教程都是拷贝粘帖，根本没有戳中问题的核心要点，比如在路径规划这里面，很多教程连坐标系这么重要的东西都没有提及。我们这个简单的路径规划example，是基于自动驾驶场景的，那么我们就以自动驾驶场景为例子进行。

首先得普及一下无人驾驶中采用的无人车坐标系，**Frenet**坐标系，这个坐标系你不要看它的定义，直接看一张图就理解了：

![](https://img-blog.csdn.net/20180622231721351?watermark/2/text/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



简单的解释Frenet坐标系如下：

以一根曲线为参照线，定义坐标系的纵轴为前进方向(s), 横轴为与s法线向量垂直的向量，从而构成Frenet坐标系，那么为什么我们规划无人车轨迹不采用迪卡尔坐标系呢？原因很简单，Frenet坐标系明显更简单，更适合我们处理无人车的路径规划问题。

我们可以将路径规划在Frenet的基础上进行分解，比如，专门求解在s方向，也就是纵向的最优路径，求解在d方向也就是横向的最优解，从而形成合成的最优化轨迹。

![](https://img-blog.csdn.net/20180622231733496?watermark/2/text/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



大概了解了一下Frenet，接下来得看一下如何实现最优化。

## 轨迹最优化的方式

《Local path planning and motion control for AGV in positioning》这篇文章中证明一个有用的结论，即，任何Jerk最优化问题的解，都可以使用一个5次多项式来表示。这里Jerk也就是加速度的加速度，描述加速度变化快慢的物理量。

<a href="https://www.codecogs.com/eqnedit.php?latex=J_t(p(t))&space;=&space;\intop\nolimits_{t_0}^{t_1}&space;p(\tau)^2d\tau" target="_blank"><img src="https://latex.codecogs.com/svg.latex?J_t(p(t))&space;=&space;\intop\nolimits_{t_0}^{t_1}&space;p(\tau)^2d\tau" title="J_t(p(t)) = \intop\nolimits_{t_0}^{t_1} p(\tau)^2d\tau" /></a>

这个公式是最小化加加速度，其目的就是在指定的时间内，完成路径的规划。文章中采用五次多项式来进行优化的步骤本文暂不做推导，感兴趣的朋友们可以查看论文推导。

![](https://img-blog.csdn.net/2018062223175747?watermark/2/text/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

要在备选集合中选择最优轨迹（即上图中的绿色轨迹），我们需要设计损失函数，对于不同的场景，损失函数也不相同，以横向轨迹为例，在较高速度的情况下，损失函数为：

<a href="https://www.codecogs.com/eqnedit.php?latex=C_d&space;=&space;k_jJ_t(d(t))&space;&plus;&space;k_tT&space;&plus;&space;k_dd_1^2" target="_blank"><img src="https://latex.codecogs.com/svg.latex?C_d&space;=&space;k_jJ_t(d(t))&space;&plus;&space;k_tT&space;&plus;&space;k_dd_1^2" title="C_d = k_jJ_t(d(t)) + k_tT + k_dd_1^2" /></a>

该损失函数包含三个惩罚项： 

* 第1项 ：惩罚Jerk大的备选轨迹； 
* 第2项：制动应当迅速，时间短； 
* 第3项：目标状态不应偏离道路中心线太远

其中 kj,ktkj,kt 和 kdkd 是这三个惩罚项的系数，它们的比值大小决定了我们的损失函数更加注重哪一个方面的优化，由此我们可以算出所有备选轨迹的损失，取损失最小的备选轨迹作为我们最终的横向轨迹。

值得注意的是，以上的损失函数仅适用于相对高速度的场景，在极端低速的情况下，车辆的制动能力是不完整的，我们不再将d表示为关于时间t的五次多项式，损失函数也会略有不同，但是这种基于有限采样轨迹，通过优化损失函数搜索最优轨迹的方法仍然是一样的，在此不再赘述。



## 轨迹生成的约束条件

在进行路径规划的过程中，除了上述损失函数的约束条件以外，还有许多约束条件。比如：

- 不能碰撞障碍物;
- 不能超过允许的最大速度;
- 不能超过允许的最大加速度;
- 转弯半径或者说曲率不能超过允许的最大值

因此，在进行路径优化的过成中也必须要加上这几条限制条件。其实这本身也是一个及其复杂的东西，关于背后的障碍物检测等部分我们不做太深入的考虑，假设我们在已经知道障碍的情况之下，如何进行路径规划呢？



## 基于Frenet参照系的路径规划算法仿真



本项目的最终目的是实现一个可用的路径规划算法，但是呢，穷苦的我们既没有无人车也没有酷选的仿真模拟器，怎么办？自己画点吧！

假如我们有几个障碍物，蓝色的点，一段红色的全局路线，比如这样：

![](https://img-blog.csdn.net/20180622231827810?watermark/2/text/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



我们现在的任何是，如何采用路径规划算法，从出发点到重点，尽量的切合全局路线，同时避开障碍物？

最终我们的规划效果如下图：

![](https://img-blog.csdn.net/20180622231847188?watermark/2/text/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



运行代码：

```
python3 path_planning.py
```

代码实现已经具有了相对详细的注释，关于代码中不懂的问题，欢迎大家留言。

