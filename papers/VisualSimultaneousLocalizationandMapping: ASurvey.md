###Abstract
视觉SLAM（同时定位和映射）是指问题
使用图像作为外部信息的唯一来源，以便建立机器人，车辆或移动照相机在环境中的位置，同时构建探索区域的表示。
SLAM是机器人自主性的重要任务。
目前，当激光或声纳等距离传感器被用来构建小静态环境的二维地图时，SLAM的问题被认为是解决的。
然而，对于动态，复杂和大规模的环境，使用视觉作为唯一的外部传感器，SLAM是一个活跃的研究领域。
视觉SLAM中使用的计算机视觉技术，诸如检测，显着特征的描述和匹配，图像识别和检索等，仍然是可以改进的。
这篇文章的目的是为视觉SLAM领域的新研究人员提供一个对现状进行简单而易于理解的综述。
###1.Introduction
移动机器人的自主导航问题分为三个主要方面：定位，制图和路径规划（Cyrill 2009）。
定位包括以确切的方式确定机器人在环境中的当前姿态。绘图将环境的部分观测结果整合到一个一致的模型中，路径规划确定了地图中通过环境进行导航的最佳路线。   
最初，绘图和本地化是独立研究的，后来得到了认可他们是依赖的。这意味着，为了在环境中精确地定位，需要正确的地图，但是为了构建好的地图，当元素被添加到地图时，必须正确定位。
目前，这个问题被称为同时定位和映射（SLAM）。当摄像机被用作唯一的外接式传感器时，它被称为视觉SLAM。
还使用术语基于视觉的SLAM（Se et al.2005; Lemaire et al.2007）或vSLAM（Solà2007）。
在这篇文章中使用术语视觉SLAM，因为它是最知名的。视觉SLAM系统可以补充来自本体感应传感器的信息，旨在提高准确性和鲁棒性。
这种方法被称为视觉惯性SLAM（Jones and Soatto 2011）。
然而，当视觉被用作唯一的感知系统时（不使用从机器人测距仪或惯性传感器中提取的信息），它可以被称为vision-only视觉SLAM（Paz等，2008; Davison等，2007）或camera-only SLAM（米尔福德和惠氏2008）。  
在以下情况下工作时，许多视觉SLAM系统失败：在外部环境，动态环境，具有太多或很少显着特征的环境中，在大规模环境中，摄像机的不稳定移动期间以及传感器的部分或全部遮挡发生时。
一个成功的视觉SLAM系统的关键是能够正确运行，尽管这些困难。  
SLAM的重要应用是针对在未经修复的越野地形上自动驾驶汽车（Thrun等，2005a）; 高风险或困难航行环境的救援任务（Thrun 2003;Piniéset al。2006）;
行星，空中，陆地和海洋勘探（Olson等人2007; Artieda等人2009; Steder等人2008; Johnson等人2010）;
虚拟对象包含在真实场景中的增强现实应用（Chekhlovet al。2007; Klein and Murray 2007）; 视觉监视系统（Mei et al。2011）;
医疗（Auat等，2010; Grasa等，2011）等等。  
在这篇文章中，介绍了视觉SLAM的详细研究，以及最近的贡献和当前的各种问题。
此前，Durrant和Bailey提出了一个教程，分为两部分，总结了SLAM问题（Durrant和Bailey 2006; Bailey和Durrant 2006）。
后面的教程描述了以使用激光测距传感器为中心的作品，以概率方法构建2D地图。同样，Thrun和Leonard（2008）介绍了SLAM问题，分析了三种解决方案（第一种基于扩展卡尔曼滤波器，另外两种使用基于图和粒子滤波器的优化技术），并提出了分类的问题。
尽管如此，上述文章并没有把视力作为唯一的外部传感器。另一方面，Kragic和Vincze（2009）在一般的背景下对机器人的计算机视觉进行了回顾，
考虑了视觉SLAM问题，但并没有详细描述本文的意图。  
本文的结构如下：
* 第2节一般性地描述了SLAM问题。
* 第3节讨论了摄像机作为唯一的外部传感器，并提到了这些系统的弱点。
* 第4节描述了可以提取的显着特征的类型和用于实现对图像可能遭受的各种转换的不变性的描述符。
* 第5节涉及图像匹配和数据关联问题。
* 第6节详细回顾了解决视觉SLAM问题的不同方法，并讨论了每个方法的弱点和长处。 描述观察世界的不同方式
教派。
* 7.第8节提供了进一步调查的结论和潜在问题。 最后一节介绍参考书目。



如果相机校准是离线执行的，那么假定相机的固有属性在SLAM系统的整个操作期间不会改变。
这是最受欢迎的选项，因为它减少了在线计算的参数数量。
然而，由于环境的某些环境因素，例如湿度或温度，内在相机信息可能会改变。 此外，在真实世界条件下工作的机器人可能会受到撞击或损坏，这可能导致以前获得的校准无效（Koch et al。2010）。  
立体配置（双目，三目或多个摄像机，其视野部分重叠）具有能够通过三角测量方法（Hartley and Sturm，1997）轻松而准确地计算场景中地标的真实三维位置的优势。 ，这是在视觉SLAM问题中非常有用的信息。 Konolige和Agrawal（2008），Konolige等人的作品（2009），Mei等人（2009）代表了目前最有效的双目立体声SLAM系统。当使用单个摄像机进行本地化和地图绘制时，地图将会受到尺度模糊问题的困扰（Nistér2004; Strasdat et al。2010a）。
为了从单个相机获得3D信息，取决于相机的先验知识存在两种情况。这些是：
* （a）在知道的情况下
内在参数;采用这种替代方法，环境结构和外部参数以不确定的尺度因子被恢复。如果空间中两点之间的实际距离是已知的，则确定尺度;
* （b）只有信件是已知的;在这种情况下，重建是由投影变换组成的。

###2 Simultaneous localization and mapping
在1985 - 1990年期间，Chatila和Laumond（1985）和Smith等人（1990）提出同时进行测绘和定位。过了一段时间，这个问题得到了SLAM（同时定位和映射）的名字。读者可以参考Durrant和Bailey（2006），Bailey和Durrant（2006）的教程，详细描述SLAM问题的历史。
在Newmanet等人的一些出版物中。 （2002），Andrade和Sanfeliu（2002）也被称为CML（并行映射和本地化）。SLAM或CML是指一个实体（机器人，车辆，甚至是一个人携带传感器设备的中央处理单元）有能力建立一个访问环境的全球地图，同时利用这个地图 随时演绎自己的位置。

为了从环境中构建地图，该实体必须具有传感器，以使其能够感知并获得来自周围世界的元素的测量结果。
这些传感器分为外感觉和本体感受。
在探测传感器中，有可能发现：声纳（Tardós等人2002; Ribas等人2008），射程激光（Nüchter等人2007; Thrun等人2006），照相机（Se等人2005; Lemaire 2007; Davison 2003; Bogdan等，2009）和全球定位系统（GPS）（Thrun等，2005a）。
所有这些传感器都是嘈杂的，并具有有限的范围能力。另外，使用前面提到的前三个传感器，只能获得当地的环境观。
激光传感器和声纳允许精确和非常密集的环境结构信息。
不过，它们存在以下问题：在高度混乱的环境中或在识别物体方面不适用;两者都昂贵，沉重，由大件设备组成，使得它们难以用于机载机器人或类人机器人。另一方面，GPS传感器在狭窄的街道（城市峡谷），水下，其他星球上效果不佳，有时在室内不可用。  
本体感应传感器允许实体获得速度，位置等测量值改变和加速。一些例子是：编码器，加速度计和陀螺仪。这些允许通过航位推算导航方法（也称为推测推算）获得对实体运动的增量估计，但是由于它们固有的噪声，它们不足以一直准确地估计实体的位置 ，因为错误是累积的。   
正如一些调查所证明的（Castellanos et al。2001; Majumder et al。2005; Nützi等人 2010），为了保持对机器人位置的精确和可靠的估计，需要使用来自多个感知传感器的信息的融合。 但是，增加传感器会增加系统的成本，重量和功率要求; 因此，调查一个实体如何定位自己并创建一个只有摄像头的地图是非常重要的。
###3 Cameras as the only exteroceptive sensors
在过去的10年中，发表的文章反映了将视觉作为解决SLAM问题的唯一外部感觉知觉系统的明显倾向（Paz等人2008; Davison等人2007; Klein和Murray 2007;Sáez和Escolano 2006; Piniés和Tardós2008）。
造成这一趋势的主要原因是基于摄像机的系统获取距离信息的能力，以及检索环境的外观，颜色和纹理的能力，使机器人有可能整合其他高级任务，如检测和识别 人和地方。此外，相机更便宜，更轻，功耗更低。不幸的是，由于以下原因，数据可能存在错误：照相机分辨率不足，照明变化，缺乏纹理的表面，由于快速移动而导致图像模糊等因素。  
关于视觉导航的第一部作品是基于双目立体配置（Seet al。2002; Olson et al。2003）。然而，在许多情况下，由于成本高，很难使用双目或三目立体相机。另一种方法是使用一对单眼相机（例如网络摄像头），从而导致考虑不同的方面，
例如：
* （a）通过使用硬件或软件的相机同步，
* （b）每个CCD传感器对 颜色和亮度，以及（c）根据所选择的几何方案（平行或会聚轴）的机械对准。  
作品还存在利用多相机钻机的具有或不具有在视图之间的重叠（Kaess和2010 Dellaert;卡雷拉等人2011）和照相机的特殊透镜如广角或全向（（Davison等人，2004。） Scaramuzza和Siegwart，2008），
目标是增加视觉范围，从而在一定程度上降低了姿态估计的累积误差。
最近，RGB-D（彩色图像和深度图）传感器已被用于绘制室内环境（Huang et al。2011），这被证明是SLAM应用的一个有前途的替代方案。

独立于所使用的配置，相机必须进行校准（手动离线或自动在线）。校准估计内在和外在参数，第一个取决于相机的几何（焦距和主点），而第二个取决于相机在空间的位置（相对于某个坐标系的旋转和平移）。
必要的参数通常是从一组包含的图像中估算出来的
棋盘格校准图案的多个视图，将图像的坐标与真实世界坐标关联（Hartley and Zisserman 2003）。许多工具可以执行校准过程，其中一些是：OpenCV（2009）（基于Zhang算法（Zhang 2000）），Matlab的相机校准工具箱（Bouguet 2010），Tsai相机校准软件Willson 1995），OCamCalib全向摄像机工具箱（Scaramuzza 2011）和多摄像机自校准来校准多台摄像机（至少3台）（Svoboda 2011）。  

如果相机校准是离线执行的，那么假定相机的固有属性在SLAM系统的整个操作期间不会改变。
这是最受欢迎的选择，因为它减少了在线计算的参数数量。
但是，由于环境的一些环境因素，例如湿度或温度，内在相机信息可能会改变。
此外，在真实世界条件下工作的机器人可能会受到撞击或损坏，这可能导致以前获得的校准无效（Koch et al。2010）。  
立体配置（双目，三目或多个摄像机，其视野部分重叠）具有能够通过三角测量方法（Hartley and Sturm，1997）轻松而准确地计算场景中地标的真实三维位置的优势。 ，这是在视觉SLAM问题中非常有用的信息。
Konolige和Agrawal（2008），Konolige等人的作品（2009），Mei等人（2009）代表了目前最有效的双目立体声SLAM系统。当使用单个摄像机进行本地化和地图绘制时，地图将会受到尺度模糊问题的困扰（Nistér2004; Strasdat et al。2010a）。
为了从单个相机获得3D信息，取决于相机的先验知识存在两种情况。
这些是：
 （a）仅具有内在参数的知识;采用这种替代方法，环境结构和外部参数以不确定的尺度因子被恢复。如果空间中两点之间的实际距离是已知的，则确定尺度;和
 （b）只有信件是已知的;在这种情况下，重建是由投影变换组成的。
 
 自单摄像头SLAM或MonoSLAM（Davison 2003）出现以来，利用一个摄像头的想法变得流行起来。 这也可能是因为通过手机，个人数字助理或个人计算机访问单个摄像机比立体对更容易。 
 这种单眼方法在硬件和处理时间方面提供了非常简单，灵活和经济的解决方案。

单目SLAM是仅有轴承SLAM的特例。
后者是一个部分可观察的问题，传感器不能从简单的观察提供足够的信息来确定地标的深度。 这导致了一个具有里程碑意义的初始化问题，其中解决方案可以分为两类：延迟和未延迟（Lemaire et al.2007; Vidal et al.2007）。 
必须执行跨越多个观测值的显着特征以从单个相机获得三维信息。

尽管对视觉SLAM做出了许多贡献，但仍存在许多问题。 为视觉SLAM问题提出的解决方案在第二部分进行了审查。 6.许多视觉SLAM系统在探索环境时（或在视觉复杂的环境中完全失败）遭受大量累积错误，这导致对机器人位置的估计不一致以及完全不协调的地图。
存在三个主要原因：

首先，通常假定相机运动是平滑的，并且会有（Davison 2003;Nistéret al。2004）的一致性，
但总的来说这不是事实。上述假设与显着特征检测器的选择以及使用的匹配技术高度相关。由于传感器的快速移动（例如，由于振动或快速方向改变）（Pupilli和Calway2006），当拍摄具有小纹理的图像或由于传感器的快速移动而模糊时，这引起照相机位置的不准确。
当相机由人，人形机器人和四旋翼直升机等携带时，这些现象是典型的。
在一定程度上缓解这个问题的一种方法是使用关键帧（见“附录I”）（Mouragnon et al.2006; Klein and Murray 2008）。或者，Pretto
等人。 （2007）和Mei和Reid（2008）分析了真实视觉追踪的问题由于离焦相机而造成的模糊图像序列上的时间。
其次，大多数研究者假定探索的环境是静态的他们只包含固定的和刚性的元素; 大部分的环境包含运动中的人物和物体。 如果这不考虑，移动的元素会引发错误的匹配，因此会产生不可预知的错误系统。这个问题的第一个方法是由Wang等人提出的。 
（2007）; Wangsiripitak和Murray（2009）; Migliore等人 （2009年），以及林和王（2010年

第三，世界在视觉上是重复的。 有很多类似的纹理，比如
重复建筑元素，叶子和砖或石头的墙壁。 在城市户外环境中也会出现一些物体，如交通信号。
这使得很难识别以前探索过的地区，也难以在大面积的土地上进行SLAM。

###4 Salient feature selection