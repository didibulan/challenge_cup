# 笨笨展示代码
* **❓与别人合作时正确且安全且建议的git命令顺序**  
🤗hwx:建议流程为:```git status```(检查目前根在哪里 以及我修改了哪些文件 要提交哪些)->```git stash``` (暂存所有更改)->```git pull``` (拉取最新更新 看看有没有别人提交了)->```git stash pop```(然后处理可能存在的冲突)->最后```git add``` 你需要的文件->```git commit -m```->```git push```(完成本次提交)  
👿2026/5/6差点在head游离分支里面提交修改导致更改全部丢失，然后还把修改提交到submodule了(~~属实非常丢人了~~)

hi from jeff - test push vision 2.0
