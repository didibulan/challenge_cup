# 挑战杯月球车相关信息
## 工程信息
- 主控  
底盘喵板  
已使用串口：uart7,fdcan1,fdcan2,fdcan3
机械臂喵板
已使用串口：uart10,fdcan1,fdcan2,fdcan3
- 底盘  
轮毂电机m3508 4  
履带电机m3508 2  
摇臂电机dm4340 4
- 机械臂  
dmj10010 1  
dm6248 2  
dm4340 3  
dm4310 3  
3507 1（夹爪电机）  
[所有达妙旗下产品开源代码和说明书都可以在这里找到](https://so.gitee.com/?q=%E8%BE%BE%E5%A6%99%E7%A7%91%E6%8A%80&svcp_stk=1_mxtgDcwDqnwfoN4Ye5gBCWi1-quBQ2zNMsjYOqJCi8PngkerTayz01A0c1vjYguz-V-qHx-JAiQY8vdftKqYCW_vtv95AOT1Y8OKKh8YE_68xvjCzX8XFSe9pKsq2sHm5canUkm2xImIzfL5Yi1UKgWal1HIX1WGyBNUC8i_S8idi1qOBxTJopBLXyWuqeazMtH9tUj05XtGn_QpcgjEzA%3D%3D)  
## 学习方向  
- B站江科大标准库和野火电子hal库自己看情况学习，各有各的优势和好处（目前车辆代码用的是hal库）  
- git仓库操作，具体可以看[廖雪峰老师的博客](https://liaoxuefeng.com/books/git/introduction/index.html)，介绍比较详细  
- 我新建了[代码仓库](https://github.com/didibulan/challenge_cup)，上传之后会另外通知，应该会在最近几天
- 工具链配置  
我们现在用的是clion——ozone工具链，具体配置方法可以参考[B站keysking](【爽！手把手教你用CLion开发STM32【大人，时代变啦！！！】】https://www.bilibili.com/video/BV1pnjizYEAk?vd_source=f2249b199342bf659d4ccf920f285b9c)
## 注意，关于什么问题我可以解答，什么我太不愿意解答（看到不愿意解答的问题我可能会已读不回）
- 代码问题  
可以问我库里的哪个函数有什么作用，大部分函数都会有标注，没有标注而且你看不懂的可以来问我，这个函数的具体功能，没有了这个函数会发生什么事情之类的，我都会尽力解答  
**不要问外设功能问题，比如uart是干嘛的，spi是干嘛的，我可能只会回“通信协议”这四个字**
- 技术问题，上位机问题
**不要问环境配置问题，比如代码运行无法解析，大概率环境出问题了，因为根据个人看的教程不同和配置不同我没法给出准确的建议，建议自己解决。自己解决不了的就寻求淘宝**