### How to add new MCU interfaces in current architecture

This guide covers, how to extend MCU functionality, using STM32CubeMX code generator.
As an example, firstly will be configured I2C interface.

### Code generation

In first four steps, configure code generator:
 1. Create new project, using repository .ioc file
 2. Enable interface with required setting
 3. [Optional] Enable interrupts
 4. Select "Copy only necessary..." and mark "Generate peripheral as a pair..."

Create project:

<img src=".images/i2c_part1.png" width="400">

Configure peripheral:

<p float="left">
  <img src=".images/i2c_part3.png" width="500" />
  <img src=".images/i2c_part4.png" width="495" /> 
</p>

Select "Copy only necessary..." and mark "Generate peripheral as a pair...":
<img src=".images/i2c_part5.png" width="500">

In the next steps, manually will be copied all necessary code parts.
Those are:
 1. Interface handler `I2C_HandleTypeDef hi2cx`
 2. MX init function `MX_I2Cx_Init(void)`
 3. HAL MSP init function `HAL_I2C_MspInit(I2C_HandleTypeDef hi2c)`
 4. [Optional] Interrupt handlers `I2Cx_EV_IRQ_Handler(void)`, `I2Cx_ER_IRQ_Handler(void)`

From `i2c.h` and `i2c.c` move `I2C_HandleTypeDef hi2cx` and `MX_I2Cx_Init(void)` to the `peripheral.h` and `peripheral.c`

<p float="left">
  <img src=".images/i2c_part7.png" width="200" />
  <img src=".images/i2c_part8.png" width="510" /> 
  <img src=".images/i2c_part9.png" width="510" /> 
</p>

From `i2c.c` move `HAL_I2C_MspInit(I2C_HandleTypeDef hi2c)` to the `stm32f4xx_hal_msp.c`

<img src=".images/i2c_part10.png" width="500">

[Optional] From `stm32f4xx_it.h` and `stm32f4xx_it.c` (generated) move interrupts handlers to the `stm32f4xx_it.h` and `stm32f4xx_it.c` (Robotont)

<p float="left">
  <img src=".images/i2c_part11.png" width="500" />
  <img src=".images/i2c_part12.png" width="490" /> 
</p>

### Implementing interface wrapper

To isolate generated content within the Robotont source code, we create a simple `ic2if` module. We will only wrap the `HAL_I2C_Mem_Write` function. There is no need to implement functionality that is not currently in use; it can always be done in the future.

We will require an init function to initialize interfaces and a `memoryWrite` function to communicate with external devices, such as the OLED SSD1306.

```c
void i2cif_init(void)
{
        
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
}

void i2cif_memoryWrite(I2C_HandleTypeDef *i2c_handler, uint16_t slave_addr, uint16_t mem_addr, uint16_t mem_size,
uint8_t *ptr_data, uint16_t data_size, uint32_t timeout_ms)
{
    (void)HAL_I2C_Mem_Write(i2c_handler, slave_addr, mem_addr, mem_size, ptr_data, data_size, timeout_ms);
}
```

If you don't plan to use interrupts, you are good to go with that implementation. But if you are planning to use them at the upper levels, then you need to do a few tricks:

 1. Define a function pointer that will be called when an interrupt occurs.
 2. Define a function that sets that function pointer. Use it somewhere in the upper level as needed.
 3. When an interrupt occurs, this function will be called.

```c
static FunctionPointerType error_callback = NULL; 

void i2cif_setErrorCallback(FunctionPointerType callback)
{
    error_callback = callback;
}

void HAL_I2C_ErrorCallback(FunctionPointerType *i2c_handler)
{
    if (error_callback != NULL)
    {
        error_callback(i2c_handler);
    }
}
```

In this approach, there is no need to import application-level components into the interface level. Consequently, this module will remain self-compilable and independent of other modules.
