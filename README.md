# AGVScheduling

## 如何运行

本项目主要使用python语言，版本是Python 3.8.5

### 安装环境

进入到AGVScheduling下使用

```bash
pip install -r requirements.txt
```

安装python需要的包

### 编译cython包

进入c_modules目录然后编译cython文件

```bash
cd c_modules
python setup.py build_ext --inplace
```

### 运行

将VnSimulator放到对应的文件夹下，然后执行根目录下的

```bash
python controller.py
```

即可


## 配置文件

根据 **如何配置文件.docx** 可以修改 **config.json** 中的内容进行配置；也可以根据 **controller.py** 中的 **parser** 进行修改
