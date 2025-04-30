# 头文件代码自动生成

首先下载[mavlink](https://github.com/mavlink/mavlink/)仓库代码，MAVLink仓库代码提供了GUI或者命令行两种方式生成不同语言的”Header"，最方便的就是运行GUI，可以通过如下命令行：

```shell
$ python mavgenerate.py
```

接下来根据GUI界面提示即可，根据你的编程语言选择生成不同语言版本的库。

或者直接运行如下命令行：

```shell
$ python pymavlink/tools/mavgen.py --lang C --wire-protocol 2.0  --output generated message_definitions/v1.0/common.xml
```



# 编写头文件生成模板



## 兼容性

​        消息解析的本质是将一段接收到的字节流数据根据**消息结构体模板**进行**数据类型强转**。如果一个消息增加了一些内容，那如何前融旧版消息呢？

​        我们可以在消息定义时，将新增内容添加在`<extensions/>`后面，那么在生成代码时，会强制将新添加的内容放到结构体最后面，这样保证了消息内容在扩展前与扩展后生成的**结构体最前面字段**的完全一致。

​		例如AUTOPILOT_VERSION消息定义中，就包含了`<extensions/>`。

```xml
    <message id="148" name="AUTOPILOT_VERSION">
      <description>Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.</description>
      <field type="uint64_t" name="capabilities" enum="MAV_PROTOCOL_CAPABILITY" display="bitmask">Bitmap of capabilities</field>
      <field type="uint32_t" name="flight_sw_version">Firmware version number</field>
      <field type="uint32_t" name="middleware_sw_version">Middleware version number</field>
      <field type="uint32_t" name="os_sw_version">Operating system version number</field>
      <field type="uint32_t" name="board_version">HW / board version (last 8 bits should be silicon ID, if any). The first 16 bits of this field specify https://github.com/PX4/PX4-Bootloader/blob/master/board_types.txt</field>
      <field type="uint8_t[8]" name="flight_custom_version">Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.</field>
      <field type="uint8_t[8]" name="middleware_custom_version">Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.</field>
      <field type="uint8_t[8]" name="os_custom_version">Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.</field>
      <field type="uint16_t" name="vendor_id">ID of the board vendor</field>
      <field type="uint16_t" name="product_id">ID of the product</field>
      <field type="uint64_t" name="uid">UID if provided by hardware (see uid2)</field>
      <extensions/>
      <field type="uint8_t[18]" name="uid2">UID2</field>
    </message>
```



在`<extensions/>`后面添加了`uid2`，那么在生成代码时，会强制将`uid2`字段放到结构体最后面，保证前面字段的一致性。

```c
typedef struct __mavlink_autopilot_version_t {
 uint64_t capabilities; /*<  Bitmap of capabilities*/
 uint64_t uid; /*<  UID if provided by hardware (see uid2)*/
 uint32_t flight_sw_version; /*<  Firmware version number*/
 uint32_t middleware_sw_version; /*<  Middleware version number*/
 uint32_t os_sw_version; /*<  Operating system version number*/
 uint32_t board_version; /*<  HW / board version (last 8 bits should be silicon ID, if any). The first 16 bits of this field specify https://github.com/PX4/PX4-Bootloader/blob/master/board_types.txt*/
 uint16_t vendor_id; /*<  ID of the board vendor*/
 uint16_t product_id; /*<  ID of the product*/
 uint8_t flight_custom_version[8]; /*<  Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.*/
 uint8_t middleware_custom_version[8]; /*<  Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.*/
 uint8_t os_custom_version[8]; /*<  Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.*/
 uint8_t uid2[18]; /*<  UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)*/
} mavlink_autopilot_version_t;
```

