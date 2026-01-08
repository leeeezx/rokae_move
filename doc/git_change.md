**git 注意事项**

2026-01-08，于win中，修改了remote branch的名称：

* impedence\_test -> main
* master -> legacy

**注意**：在Ubuntu中，需要手动修改对应关系！

步骤如下（以`impedence_test`重命名为`main`为例）：

1. 重命名本地分支，作用：将你本地的 `impedence_test` 改名为 `main`。

```Shell
git branch -m impedence_test main
```

2. 获取远程更新，作用：让本地 Git 知道远程仓库现在发生了什么变化（即：发现了远程的 `main`）。

```Shell
git fetch origin
```

3. 重新关联远程分支，作用：告诉本地的 main 分支，以后它的“上游”就是远程的 origin/main

```Shell
git branch -u origin/main main
```

4. 更新远程HEAD指针，作用：更新本地对远程默认分支（HEAD）的认知。

```Shell
git remote set-head origin -a
```

