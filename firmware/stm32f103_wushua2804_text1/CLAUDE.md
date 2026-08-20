# STM32F103 FOC 无刷电机控制器

## 编码规范

- **源文件编码**：`.c` 和 `.h` 文件使用 GB2312 编码（Keil 工程要求）
- **Claude Code 工具链限制**：Read/Edit/Write 工具固定使用 UTF-8，无法直接正确读写 GB2312 文件中的中文

### 编辑 .c/.h 文件的强制工作流

**第一步：编辑前，先将目标文件从 GB2312 转为 UTF-8**
```python
python -c "
import sys, os
for f in sys.argv[1:]:
    with open(f, 'rb') as fh: raw = fh.read()
    with open(f, 'w', encoding='utf-8') as fh: fh.write(raw.decode('gb2312'))
    print('GB2312->UTF-8:', f)
" app/can_app.c app/can_app.h
```

**第二步：正常用 Edit/Write 工具编辑文件，中文注释可直接用 UTF-8 写入**

**第三步：所有编辑完成后，将文件从 UTF-8 转回 GB2312**
```python
python -c "
import sys, os
for f in sys.argv[1:]:
    with open(f, 'r', encoding='utf-8') as fh: text = fh.read()
    with open(f, 'wb') as fh: fh.write(text.encode('gb2312'))
    print('UTF-8->GB2312:', f)
" app/can_app.c app/can_app.h
```

> **为什么不能跳过第一步？** 如果直接在 GB2312 文件中用 Edit 添加 UTF-8 中文，文件会变成混合编码（旧注释=GB2312，新注释=UTF-8），导致中文注释部分正常、部分乱码。

