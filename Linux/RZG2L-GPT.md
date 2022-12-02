# RZG2L General PWM Timer (GPT)

- [1. overview](#1-overview)
- [2. 出厂源码](#2-出厂源码)
- [3. cpg reset报错](#3-cpg-reset报错)
- [4. 双通道同时输出](#4-双通道同时输出)
  - [4.1. 设备树](#41-设备树)
  - [4.2. 测试](#42-测试)
  - [4.3. 驱动](#43-驱动)

## 1. overview

This LSI(Large Scale Integration) has a general purpose PWM timer (GPT) composed of 8 channels of 32-bit timer (GPT32E).

这里提到的channel和驱动源码中提到的channel_A,channel_B不是一个概念

- 这里的channel指不同定时器，可以独立设置周期、占空比
- 驱动中的channel_A,channel_B指一个定时器的两个通道，只可设置占空比，AB通道的周期相同

## 2. 出厂源码

目前用的 <https://github.com/renesas-rz/rz_linux-cip/tree/rzg2l-cip41>

- `drivers/pwm/gpt-rzg2l.c`
  - `static int rzg2l_gpt_probe(struct platform_device *pdev)`
    - 设备树和驱动匹配之后会执行probe，其中有关于channel_A,channel_B的描述，但是没有both_AB
    - 这个版本的驱动不支持AB两个通道同时输出

参考 <https://raw.githubusercontent.com/renesas-rz/rz_linux-cip/rzg2l-cip54/drivers/pwm/gpt-rzg2l.c> 修改

## 3. cpg reset报错

gpt3和gpt6同时打开，内核启动阶段会报错failed to get cpg reset

```log
[root@okg2l ~ ]# dmesg |grep gpt
[    0.090055] gpt-rzg2l 10048300.gpt: RZ/G2L GPT Driver probed
[    0.090361]  rzg2l_gpt_probe+0x90/0x378
[    0.090409]  rzg2l_gpt_driver_init+0x18/0x20
[    0.090453] gpt-rzg2l 10048600.gpt: failed to get cpg reset
[    0.090506] gpt-rzg2l: probe of 10048600.gpt failed with error -16
```

错误-16 `#define EBUSY 16 /* Device or resource busy */`

在`drivers/pwm/gpt-rzg2l.c`的`rzg2l_gpt_probe`函数中可以看到关于cpg的配置信息

在probe过程中，需要申请rstc，即复位控制器资源，但是，只有第一次gpt3能成功获取，第二次gpt6就不能再获取这个资源

而从实验现象看，只需要复位一次，因此，对于产生错误的情况不return，继续执行

```c
static int rzg2l_gpt_probe(struct platform_device *pdev)
{
  ...

  rzg2l_gpt->rstc = devm_reset_control_get(&pdev->dev, NULL);
  if (IS_ERR(rzg2l_gpt->rstc)) {
    dev_err(δpdev->dev, "failed to get cpg reset\n");
    // return PTR_ERR(rzg2l_gpt->rstc);
  }

  ...
}
```

[详细分析](./Linux-reset-framework.md)

## 4. 双通道同时输出

### 4.1. 设备树

```c
&pinctrl{
  gpt6_pins: gpt6 {
    groups = "gpt6_a_a", "gpt6_b_a";
    function = "gpt6";
  };
};
&gpt6 {
  pinctrl-0 = <&gpt6_pins>;
  pinctrl-names = "default";
  channel="both_AB";
  status = "okay";
};
```

### 4.2. 测试

pwm在文件系统中的接口

- 确认pwmchip，根据软链接指向，pwmchip1对应gpt6

  ```bash
  ls -al /sys/class/pwm
  ```

- 修改周期，单位ns

  ```bash
  echo 0 > /sys/class/pwm/pwmchip1/export
  echo 1000000 > /sys/class/pwm/pwmchip1/pwm0/period
  echo 1 > /sys/class/pwm/pwmchip1/pwm0/enable
  ```

- 修改占空比，修改channel_A的占空比，单位ns，要小于period

  ```bash
  echo 800000 > /sys/class/pwm/pwmchip1/device/buffA0
  echo 200000 > /sys/class/pwm/pwmchip1/device/buffB0
  ```

### 4.3. 驱动

在文件系统中添加接口

```c
static ssize_t buffA0_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
static ssize_t buffA0_show(struct device *dev, struct device_attribute *attr, char *buf)

static DEVICE_ATTR_RW(buffA0);
static struct attribute *buffer_attrs[] = {
  &dev_attr_buffA0.attr,

  ...
}
```

- [使用 /sys 文件系统访问 Linux 内核](https://blog.csdn.net/yuantian2987/article/details/12844061)
