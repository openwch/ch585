#ifndef __WCH_TOUCH_H__
#define __WCH_TOUCH_H__
//------------------------------------------

//---------------filter-----------------
#define FILTER_MODE_3    	      3       //--滤波器模式3, 可输出多个按键--
//---------------single key mode-----------------
#define TKY_SINGLE_KEY_MULTI      0       //--多按键输出，即超过阈值的按键都会触发
#define TKY_SINGLE_KEY_MAX        1       //--最大值单按键输出，即超过阈值的按键中只上报变化量最大的按键
#define TKY_SINGLE_KEY_MUTU       2       //--互斥单按键输出，即当前按键释放后才会上报下一个变化量最大的按键，
                                          //  否则其他按键无论变化量多大都不上报

//--------------lib param-----------------
#define TKY_BUFLEN  			  48
#define TOUCH_VER_FILE  "CH585_CT_TOUCH_LIB_V3.0.3"
extern const uint8_t TOUCH_VER_LIB[];  // LIB version

//---------------------------------------
typedef struct
{
	uint8_t maxQueueNum;				    //--测试队列数量--
	uint8_t singlePressMod;                	//--单按键模式;0-多按键;1-最大值单按键；2-互斥单按键；---
	uint8_t shieldEn;                   	//--屏蔽使能---
	uint8_t filterMode;				        //--滤波器模式--
	uint8_t filterGrade;				    //--滤波器等级--
	uint8_t peakQueueNum;                	//--按键最大偏移队列---
	uint8_t peakQueueOffset;             	//--按键最大偏移队列的偏移值---
	uint8_t baseRefreshOnPress;			    //--基线在按键按下时是否进行--
	uint8_t baseUpRefreshDouble;        	//--基线向上刷新倍速参数---
	uint8_t baseDownRefreshSlow;       		//--基线向下更新降速参数---
	uint8_t maxNoiseChCount;				//--干扰通道数量--
	uint8_t sleepChNum;						//--睡眠通道数量--
    uint32_t baseRefreshSampleNum;     		//--基线刷新采样次数--
	uint32_t *tkyBufP;					 	//--测试通道数据缓冲区指针--
}TKY_BaseInitTypeDef;

typedef struct
{
	uint16_t queueNum;                 		//--该通道在测试队列的序号--
	uint16_t channelNum;               		//--该通道对应的ADC通道标号--
	uint16_t baseLine;  	   	        	//--参考基线--
	uint16_t maxVar;		            	//--参考最大变化量--
	uint8_t sensitivityLevel;              //--灵敏度1~10--
	uint8_t hysteresisWindow;            //--迟滞比较窗口--

}TKY_ChannelInitTypeDef;

/**
 * @brief 基础初始化函数
 * 
 * 该函数用于初始化触摸系统的基础模块，通过传入基础初始化结构体参数来配置系统的核心工作参数。
 * 详见与TKY_BaseInitTypeDef，通常用于系统上电后的首次初始化。
 * 
 * @param TKY_BaseInitStruct 基础初始化结构体
 *                          - 具体结构体定义参考TKY_BaseInitTypeDef文档
 * 
 * @return uint8_t 返回初始化状态码
 *                - 0：基础初始化成功
 *                - bit1：滤波等级超出上限（最大为16）
 * 
 * @note 此函数应在其他模块初始化之前调用，确保系统基础环境就绪
 * @warning 基础初始化失败可能导致整个系统无法正常工作
 * @attention 初始化参数需根据具体硬件平台和系统需求进行配置
 * @see TKY_CHInit 通道初始化函数
 * 
 * @example
 * // 定义并填充基础初始化结构体
 * TKY_BaseInitTypeDef baseInit = {...};
 * 
 * // 执行基础初始化
 * uint8_t result = TKY_BaseInit(baseInit);
 * if (result == 0) {
 *     // 基础初始化成功，可继续初始化其他模块
 *     TKY_CHInit(chInit);
 * }
 */
extern uint8_t TKY_BaseInit(TKY_BaseInitTypeDef TKY_BaseInitStruct);

/**
 * @brief 通道初始化函数
 * 
 * 该函数用于初始化触摸系统的通道配置，通过传入初始化结构体参数来配置通道的各项工作参数。
 * 用于系统启动时或通道参数需要重新配置时的初始化操作。
 * 
 * @param TKY_CHInitStruct 通道初始化结构体
 *                        - 包含通道工作模式、参数配置等初始化信息
 *                        - 具体结构体定义参考TKY_ChannelInitTypeDef
 * 
 * @return uint8_t 返回初始化状态码
 *                - 0：通道初始化成功
 *                - bit1：转换队列编号超过最大队列数量
 *				  - bit2：基线值溢出
 *
 * 
 * @note 初始化操作通常只需在系统启动或配置变更时调用一次
 * @warning 初始化过程中通道可能暂时不可用，请勿在关键操作期间调用
 * @attention 初始化结构体中的参数需符合硬件规格要求
 * 
 * @example
 * // 定义并填充初始化结构体
 * TKY_ChannelInitTypeDef chInit = {...};
 * 
 * // 执行通道初始化
 * uint8_t result = TKY_CHInit(chInit);
 * if (result == 0) {
 *     // 初始化成功，通道可正常使用
 * }
 */
extern uint8_t TKY_CHInit(TKY_ChannelInitTypeDef TKY_CHInitStruct);

/**
 * @brief 获取当前通道数据
 * 
 * 该函数用于获取指定通道的采样数据，支持多次转换，以供后续噪声分析和数据处理。
 * 
 * @param curChNum   指定的触摸通道编号，与芯片相关
 * @param CTransN    转换次数参数，范围1~1023
 * @param CTransCC   转换控制参数，CTransCC[1:0]--控制参数;CTransCC[15:2]--保留
 * @param scanArray  输出参数，指向存储采样数据的缓冲区
 *                  - 函数将通道数据填充到此数组
 *                  - 缓冲区大小应至少能容纳dataNum个数据
 * @param dataNum 	数据数量
 *                  - 指定用于要获取的数据采样点数
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取通道数据
 *                - 1：采样数量为0
 *				  - 2：转换次数参数超出范围
 * 
 * @note 数据获取过程可能需要较长时间，具体取决于转换次数
 * @warning 确保scanArray缓冲区足够大，否则可能导致数据溢出
 * 
 * @example
 * uint16_t sampleData[10]; // 存储10个采样数据
 * // 获取触摸通道2的数据，转换5次，平均8个样本
 * uint8_t result = TKY_GetCurChannelData(2, 5, 3, sampleData, 8);
 * if (result == 0) {
 *     // 成功获取数据，可进行后续处理
 *     ProcessChannelData(sampleData, 8);
 * }
 */
extern uint8_t TKY_GetCurChannelData(uint16_t curChNum, uint16_t CTransN, 
                                     uint16_t CTransCC, uint16_t* scanArray, 
                                     uint16_t dataNum);


/**
 * @brief 噪声估计计算函数
 * 
 * 该函数通过对输入的样本数据进行统计分析，估计信号的噪声水平。
 * 同时计算样本的平均值，用于后续的信号处理分析。
 * 
 * @param samples   输入样本数据数组指针，指向待分析的原始采样数据
 * @param count     样本数量，指定samples数组中的有效数据个数
 * @param noise   	输出参数指针，用于返回所有样本的噪声水平
 * @param mean   	输出参数指针，用于返回所有样本的平均值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功
 *                - 1：队列编号无超出范围
 * 
 * @note 传入样本数据不要越界
 * @warning 确保samples数组有足够的count个有效数据，否则可能导致计算错误
 * 
 * @example
 * uint16_t data[100] = {...}; // 采样数据
 * uint16_t avg;
 * uint16_t noise = TKY_NoiseEstimate(data, 100, &avg);
 */
extern uint8_t TKY_NoiseAndMeanEstimate(uint16_t *samples, uint32_t count, uint16_t* noise, uint16_t* mean);

/**
 * @brief 轮询滤波模式3的按键输出结果
 * 
 * 该函数用于轮询并获取特定滤波模式3的各队列按键按下结果。
 * 
 * @param 无
 * 
 * @return uint16_t 返回滤波模式3的按键按下状态
 *                  - 不同位代表不同转换队列按键按下状;1-按下,0-未按下
 * 
 * @note 此函数采用阻塞轮询方式，会占用CPU资源直至此次扫描结束
 * @warning 
 * @attention 
 * 
 * @example
 * // 轮询滤波模式3按键输出
 * uint16_t keydata;
 * keydata = TKY_PollForFilterMode_3();
 * 
 * // 进行后续处理
 * if (keydata & KEY1_READY_BIT) {
 *     //KEY1被按下
 * }
 */
extern uint16_t TKY_PollForFilterMode_3(void);

/**
 * @brief 轮询CS10滤波模式下按键输出结果
 * 
 * 该函数用于轮询并获取CS10的按键按下状态。
 * 通过非阻塞方式获取滤波器的按键输出状态。
 * 
 * @param 无
 * 
 * @return uint16_t 返回CS10的按键按下状态
 *                  - 不同位代表不同转换队列按键按下状;1-按下,0-未按下
 * 
 * @note 此函数采用非阻塞方式轮询，占用CPU资源小。
 * @warning 需要高频调用，最好放在主循环中调用，否则影响按键响应的实时性
 * @attention 
 * @see TKY_SetFilterMode 相关的滤波模式设置函数
 * 
 * @example
 * // 轮询CS10滤波模式按键输出.
 * uint16_t keydata;
 * keydata = TKY_PollForFilterMode_CS10();
 * 
 * // 进行后续处理
 * if (keydata & KEY1_READY_BIT) {
 *     //KEY1被按下
 * }
 */
extern uint16_t TKY_PollForFilterMode_CS10(void);

/**
 * @brief 扫描唤醒事件
 * 
 * 该函数用于扫描系统唤醒，检测指定的唤醒条件是否满足。
 * 通常用于低功耗模式下定期检查唤醒条件，以便及时恢复正常工作状态。
 * 
 * @return uint16_t 返回唤醒状态
 *					-0:不满足唤醒条件
 *					-1:唤醒条件满足
 *					-2:睡眠通道配置出错
 * 
 * @note 此函数通常在低功耗模式的循环中调用，用于检测唤醒条件
 * @warning 扫描过程可能需要一定时间
 * 
 */
extern uint16_t TKY_ScanForWakeUp(void);

/**
 * @brief 睡眠检测细扫函数
 * 
 * 该函数用于扫描系统唤醒，在TKY_ScanForWakeUp触发的基础上进一步检测指定的唤醒条件是否满足。
 * 通常用于低功耗模式下定期检查唤醒条件，以便及时恢复正常工作状态。
 * 
 * @return uint16_t 返回唤醒状态
 *					-0:不满足唤醒条件
 *					-非0:唤醒条件满足
 * 
 * @note 此函数通常在低功耗模式的循环中调用，用于检测唤醒条件
 * @warning 扫描过程可能需要一定时间
 * 
 */
extern uint16_t TKY_PollForFilterSleep( void );

/**
 * @brief 设置当前队列睡眠状态
 * 
 * 该函数用于控制指定队列的睡眠状态，使队列进入或退出睡眠模式。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要设置睡眠状态的队列索引（0起始）
 *                   - 具体队列索引参考系统队列配置
 * @param sleepStatus 睡眠状态设置值
 *                   - 0：唤醒队列，恢复正常工作模式
 *                   - 1：使队列进入睡眠/低功耗模式
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置队列睡眠状态
 *                - 非0：错误码（如队列编号无效、状态切换失败等）
 * 
 * @note 队列睡眠状态下无法进行触摸检测
 * @warning 在需要队列工作的场景中，请确保队列处于唤醒状态
 * @see TKY_SetSleepStatusValue 系统全局睡眠状态设置函数
 * @see TKY_GetSleepStatusValue 系统睡眠状态获取函数
 * 
 * @example
 * // 使队列0进入睡眠状态
 * TKY_SetCurQueueSleepStatus(0, 1);
 * 
 * // 当需要队列工作时唤醒队列0
 * TKY_SetCurQueueSleepStatus(0, 0);
 */
extern uint8_t TKY_SetCurQueueSleepStatus(uint8_t curQueueNum, uint8_t sleepStatus);

/**
 * @brief 设置睡眠状态值
 * 
 * 该函数用于配置系统的睡眠状态值，控制触摸队列的睡眠行为。
 * 
 * @param setValue 要设置的睡眠状态值
 *               - 不同位控制不同转换队列的睡眠功能
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置睡眠状态值
 *                - 1：设置值超过队列最大长度
 * 
 * @note 睡眠状态值每一位代表一个转换队列，
 *		 睡眠状态值从低位到高位与转换队列编号从0开始由低到高一一对应
 * @warning 错误的设置可能导致触摸无法响应
 * @attention 设置前应确保了解各控制位的具体功能
 * @see TKY_GetSleepStatusValue 相关的睡眠状态获取函数
 * 
 * @example
 * // 设置系统进入浅睡眠模式
 *
 * uint16_t	SleepStatus = TKY_GetSleepStatusValue();
 * SleepStatus|= 0x0003
 * uint8_t result = TKY_SetSleepStatusValue(SleepStatus);
 * if (result == 0) {
 *     // 通道0和通道1睡眠模式设置成功
 * }
 * 
 * // 配置特定的睡眠参数
 * uint16_t sleepConfig = SLEEP_TIMEOUT_MS | WAKE_SOURCE_ENABLE;
 * TKY_SetSleepStatusValue(sleepConfig);
 */
extern uint8_t TKY_SetSleepStatusValue(uint32_t setValue);

/**
 * @brief 获取触摸系统各转换队列的睡眠状态
 * 
 * 该函数用于读取触摸当前的睡眠状态值
 * 
 * @param 无
 * 
 * @return uint16_t 返回当前系统的睡眠状态值
 * 
 * @note 睡眠状态值每一位代表一个转换队列，
 *		 睡眠状态值从低位到高位与转换队列编号从0开始由低到高一一对应
 * @warning 
 * 
 * @example
 * // 监测系统睡眠状态
 * uint16_t sleepStatus = TKY_GetSleepStatusValue();
 *
 * if ((sleepStatus & (1<<0)) != 0) {
 *     // 转换队列0处于深度睡眠状态
 * }
 */
extern uint16_t TKY_GetSleepStatusValue(void);

/**
 * @brief 设置当前队列充电时间参数
 * 
 * 该函数用于配置指定队列的充电时间相关参数。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要设置充电时间参数的队列索引（0起始）
 * @param CTransN     转换次数参数，范围1~1023
 * @param CTransCC    转换控制参数，CTransCC[1:0]--控制参数;CTransCC[15:2]--保留
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置充电时间参数
 *                - 1：队列编号超出范围
 *				  - 2：转换次数参数超出范围
 * 
 * @note 
 * @warning 
 * @attention 
 * 
 * @example
 * // 设置队列0的充电时间参数
 * uint8_t result = TKY_SetCurQueueChargeTime(0, 10, 5);
 * if (result == 0) {
 *     // 参数设置成功
 * }
 */
extern uint8_t TKY_SetCurQueueChargeTime(uint8_t curQueueNum, uint16_t CTransN, uint16_t CTransCC);

/**
 * @brief 设置组内当前队列充电时间参数
 * 
 * 该函数用于分组管理的多队列系统，可分组配置充电参数。
 * 
 * @param groupidx    组索引
 *                   - 指定要操作的组编号（0~3）
 *                   - 不同组可能对应不同的充电参数
 * @param curQueueNum 组内队列编号
 *                   - 指定组内要设置的队列索引（0起始）
 *                   - 队列编号在组内有效
 * @param CTransN     转换次数参数，范围1~1023
 * @param CTransCC    转换控制参数，CTransCC[1:0]--控制参数;CTransCC[15:2]--保留
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置充电时间参数
 *                - 1：队列编号超出范围
 *				  - 2：转换次数参数超出范围
 *				  - 3：组索引无效
 * 
 * @note 多组参数代表每个通道有多组参数，用于抗干扰。
 * @warning 
 * @see TKY_SetCurQueueChargeTime 单个队列充电时间设置函数
 * 
 * @example
 * // 设置组0内队列0的充电时间参数
 * uint8_t result = TKY_SetGroupCurQueueChargeTime(0, 0, 10, 5);
 * if (result == 0) {
 *     // 参数设置成功
 * }
 */
/**
 * !! 警示（2026-10-13 审查）：本函数与 TKY_GetGroupCurQueueChargeTime 形参顺序互反
 * （Get 为 curQueueNum 在前）。闭源库真实参数顺序未经WCH确认，按字面签名调用两者
 * 无法同时正确，集成前务必向WCH核实并对读回值做实测验证。
 */
extern uint8_t TKY_SetGroupCurQueueChargeTime(uint8_t groupidx, uint8_t curQueueNum, uint16_t CTransN, uint16_t CTransCC);

/**
 * @brief 获取当前队列充电时间参数
 * 
 * 该函数用于读取指定队列的当前充电时间参数，包括转换次数和转换控制参数。
 * 可用于参数验证、状态监测或动态调整前的参数获取。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要获取充电时间参数的队列索引（0起始）
 * @param CTransN     输出参数，指向存储转换次数参数的缓冲区
 *                   - 返回当前设置的转换次数值
 * @param CTransCC    输出参数，指向存储转换控制参数的缓冲区
 *                   - 返回当前设置的转换控制值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取充电时间参数
 *                - 1：队列编号超出范围
 * 
 * @note 获取的参数应与之前设置的参数一致，除非被其他操作修改
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * @see TKY_SetCurQueueChargeTime 相关的充电时间参数设置函数
 * 
 * @example
 * uint16_t transN, transCC;
 * uint8_t result = TKY_GetCurQueueChargeTime(0, &transN, &transCC);
 * if (result == 0) {
 *     // 成功获取参数，可用于验证或显示
 *     printf("Queue 0 - CTransN: %u, CTransCC: %x\n", transN, transCC);
 * }
 */
extern uint8_t TKY_GetCurQueueChargeTime(uint8_t curQueueNum, uint16_t* CTransN, uint16_t* CTransCC);

/**
 * @brief 获取组内当前队列充电时间参数
 * 
 * 该函数用于读取指定组内特定队列的当前充电时间参数，包括转换次数和转换控制参数。
 * 
 * @param curQueueNum 组内队列编号
 *                   - 指定组内要查询的队列索引（0起始）
 * @param groupidx    组索引
 *                   - 指定要查询的组编号（0~3）
 * @param CTransN     输出参数，指向存储转换次数参数的缓冲区
 *                   - 返回当前设置的转换次数值
 * @param CTransCC    输出参数，指向存储转换控制参数的缓冲区
 *                   - 返回当前设置的转换控制值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取充电时间参数
 *                - 1：队列编号超出范围
 *                - 3：组索引超出范围
 * 
 * @note 组内队列编号与全局队列编号可能不同，请参考系统分组策略
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * @see TKY_GetCurQueueChargeTime 单个队列充电时间获取函数
 * @see TKY_SetGroupCurQueueChargeTime 组内队列充电时间设置函数
 * 
 * @example
 * uint16_t transN, transCC;
 * // 获取组1内队列0的充电时间参数
 * uint8_t result = TKY_GetGroupCurQueueChargeTime(1, 0, &transN, &transCC);
 * if (result == 0) {
 *     // 成功获取参数
 *     printf("Group 1, Queue 0 - CTransN: %u, CTransCC: %x\n", transN, transCC);
 * }
 */
extern uint8_t TKY_GetGroupCurQueueChargeTime(uint8_t curQueueNum, uint8_t groupidx, uint16_t* CTransN, uint16_t* CTransCC);


/**
 * @brief 查询库内部状态机当前是否为空闲状态
 * 
 * @param 无
 * 
 * @return uint8_t 返回当前系统的空闲状态
 *                - 0：工作中
 *                - 1：空闲
 * 
 * @note 空闲状态的判断仅在非阻塞扫描时有意义,阻塞扫描时常为空闲。
 * @warning 
 * @see TKY_SaveAndStop 相关的触摸扫描停止函数
 * 
 * @example
 *
 * // 检查系统是否空闲，如果是则进行其他操作
 * TKY_SaveAndStop();
 * if (TKY_GetCurIdleStatus() == 1) {
 *     //其他操作
 * }
 */
extern uint8_t TKY_GetCurIdleStatus(void);

/**
 * @brief 获取当前触摸库版本号
 * 
 * 
 * @param 无
 * 
 * @return uint16_t 返回当前库版本号
 * 
 * @note 版本号在编译时确定，运行时不可修改
 * @warning 
 * 
 * @example
 * uint16_t version = TKY_GetCurVersion();
 * printf("System Version: %u\n", version);
 * 
 *若version=300，则代表版本号为3.0.0
 */
extern uint16_t TKY_GetCurVersion(void);

/**
 * @brief 获取当前队列数值
 * 
 * 该函数用于读取指定队列的当前数值，是原始采样值经扫描滤波器处理后的数值，适用于实时数据监测。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要获取数值的队列索引（0起始）
 *                   - 具体队列数量参考系统队列配置
 * @param queueValue  输出参数，指向存储滤波值的缓冲区
 *                   - 返回当前滤波器的处理值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取充电时间参数
 *                - 1：队列编号超出范围
 * 
 * @note 此函数返回的是队列的处理值，库内部参考此值输出最终按键，适用于实时监测
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * @see TKY_GetCurQueueRealVal 获取队列实时值函数
 * @see TKY_GetCurQueueBaseLine 获取队列基线值函数
 * @see TKY_GetCurQueueThreshold 获取队列阈值函数
 * 
 * @example
 * // 获取队列0的当前数值
 * uint16_t queueValue = 0;
 * uint8_t result = TKY_GetCurQueueValue(0,&queueValue);
 * 
 * if (result == 0) {
 *     // 成功获取参数,用于实时显示
 * 		printf("current value: %u\n", curQueueNum, queueValue);
 * }
 * 
 */
extern uint8_t TKY_GetCurQueueValue(uint8_t curQueueNum, uint16_t* queueValue);

/**
 * @brief 获取当前队列实时值
 * 
 * 该函数用于读取指定队列的当前实时测量值，用于实时监测、数据显示。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要获取实时值的队列索引（0起始）
 * @param queueValue  输出参数，指向存储真实值的缓冲区
 *                   - 返回当前触摸扫描的真实值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取充电时间参数
 *                - 1：队列编号超出范围
 * 
 * @note 该函数返回的是瞬时值，可能包含噪声，适用于实时显示
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * 
 * @example
 * // 获取队列0的当前实时值
 * uint16_t queueValue = 0;
 * uint8_t result = TKY_GetCurQueueRealVal(0,&queueValue);
 * 
 * if (result == 0) {
 *     // 成功获取参数,用于实时显示
 * 		printf("current value: %u\n", curQueueNum, queueValue);
 * }
 *
 */
extern uint8_t TKY_GetCurQueueRealVal (uint8_t curQueueNum, uint16_t *queueValue);

/**
 * @brief 获取当前队列滤波值
 * 
 * 该函数用于读取指定队列的当前滤波值，用于实时监测、数据显示。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要获取滤波值的队列索引（0起始）
 * @param filterValue  输出参数，指向存储真实值的缓冲区
 *                   - 返回当前触摸扫描的滤波值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取充电时间参数
 *                - 1：队列编号超出范围
 * 
 * @note 该函数返回的是瞬时值，可能包含噪声，适用于实时显示
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * 
 * @example
 * // 获取队列0的当前实时值
 * uint16_t filterValue = 0;
 * uint8_t result = TKY_GetCurQueueFilterVal(0,&filterValue);
 * 
 * if (result == 0) {
 *     // 成功获取参数,用于实时显示
 * 		printf("current value: %u\n", curQueueNum, filterValue);
 * }
 *
 */
extern uint8_t TKY_GetCurQueueFilterVal (uint8_t curQueueNum, uint16_t *filterValue);

/**
 * @brief 获取当前队列噪声值
 * 
 * 该函数用于读取指定队列的当前噪声值，用于实时监测、数据显示。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要获取滤波值的队列索引（0起始）
 * @param variance    输出参数，指向存储噪声的缓冲区
 *                   - 返回当前触摸扫描的噪声值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取充电时间参数
 *                - 1：队列编号超出范围
 * 
 * @note 该函数返回的是瞬时值，可能包含噪声，适用于实时显示
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * 
 * @example
 * // 获取队列0的当前实时值
 * uint16_t filterValue = 0;
 * uint8_t result = TKY_GetCurQueueFilterVal(0,&filterValue);
 * 
 * if (result == 0) {
 *     // 成功获取参数,用于实时显示
 * 		printf("current value: %u\n", curQueueNum, filterValue);
 * }
 *
 */
extern uint8_t TKY_GetCurQueueVariance (uint8_t curQueueNum, uint16_t *variance);

/**
 * @brief 设置当前队列阈值参数
 * 
 * 该函数用于配置指定队列的阈值参数，包括上限阈值和下限阈值。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要设置阈值的队列索引（0起始）
 *                   - 具体队列数量参考系统队列配置
 * @param threshold   上限阈值（主阈值）
 *                   - 用于设置队列的主阈值
 * @param threshold2  下限阈值（辅助阈值）
 *                   - 用于设置队列的辅助阈值
 *					 - 可以为0，为0时下限阈值由库内部根据数据
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置阈值参数
 *                - 1：列编号无效
 * 
 * @note 
 * @warning 阈值设置不当可能导致误报漏报按键，请谨慎设置
 * 
 * @example
 * // 设置队列0的主阈值为100，辅助阈值为70
 * uint8_t result = TKY_SetCurQueueThreshold(0, 100, 70);
 * if (result != 0) {
 *     // 处理设置失败情况
 * }
 */
extern uint8_t TKY_SetCurQueueThreshold(uint8_t curQueueNum, uint16_t threshold, uint16_t threshold2);

/**
 * @brief 获取当前队列阈值参数
 * 
 * 该函数用于读取指定队列的阈值参数，包括上限阈值和下限阈值。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要获取阈值的队列索引（0起始）
 *                   - 具体队列数量参考系统队列配置
 *
 * @param threshold   输出参数，指向存储上限阈值(主阈值)的缓冲区
 * @param threshold2  输出参数，指向存储下限阈值(辅助阈值)的缓冲区
 * 
 * @return uint8_t 返回操作状态或错误码
 *                - 0：成功获取阈值参数
 *                - 1：队列号超出范围
 * 
 * @note 
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * 
 * @example
 * uint16_t lowerThresh, upperThresh;
 * uint8_t status = TKY_GetCurQueueThreshold(0, &upperThresh, &lowerThresh);
 * if (status == 0) {
 *     // 成功获取阈值，进行后续处理
 * }
 */
extern uint8_t TKY_GetCurQueueThreshold(uint8_t curQueueNum, uint16_t *threshold, uint16_t *threshold2);

/**
 * @brief 设置指定队列的基线值
 * 
 * 该函数用于设置指定队列的基线参值，影响后续算法的判断和输出。
 * 
 * @param curQueueNum  指定队列编号
 *                    - 指定要设置基线值的队列索引（0起始）
 *                    - 具体队列数量参考系统队列配置
 * 
 * @param baseLineValue 要设置的基线值
 * 
 * @return uint8_t	- 0:设置成功
 *					- 1:队列号超出范围
 * 
 * @note 基线值设置后会影响后续触摸数据处理的参考点
 * @warning 错误的基线值设置可能导致按键持续触发或者按键无法触发
 * @attention 确保队列编号在有效范围内
 * 
 * @example
 * // 设置队列0的基线值为3000
 * TKY_SetCurQueueBaseLine(0, 3000);
 * 
 * // 设置队列1的基线值为3200
 * TKY_SetCurQueueBaseLine(1, 3200);
 */
extern uint8_t TKY_SetCurQueueBaseLine(uint8_t curQueueNum, uint16_t baseLineValue);


/**
 * @brief 获取当前队列基线值
 *
 * 该函数用于读取指定队列的当前基线值
 *
 * @param curQueueNum 		当前队列编号
 *                   		- 指定要获取基线值的队列索引（0起始）
 *                   		- 具体队列数量参考系统队列配置
 * @param baseLineValue    输出参数，指向存储基线值的缓冲区
 *                   	   - 返回当前查询的基线值
 *
 * @return uint8_t	- 0:设置成功
 *					- 1:队列号超出范围
 *
 * @note 获取的基线值应与之前设置的基线值一致，除非被动态修改
 * @warning 确保队列编号在有效范围内，否则可能返回错误值
 * @see TKY_SetCurQueueBaseLine 相关的基线值设置函数
 *
 * @example
 * // 获取队列0的当前基线值
 * uint16_t baseline;
 * uint8_t result = TKY_GetCurQueueBaseLine(0, &baseline);
 * if (result == 0) {
 *     // 基线值获取成功
 * }
 */
extern uint8_t TKY_GetCurQueueBaseLine(uint8_t curQueueNum, uint16_t* baseLineValue);

/**
 * @brief 设置基线刷新采样点数
 * 
 * 该函数用于配置基线刷新周期内的采样点数，即每采newValue个点刷新一次基线。
 * 采样点数与刷新率成反比关系，需要在稳定性和实时性之间权衡。
 * 
 * @param newValue 采样点数设置
 *               - 值越大：单次刷新采集更多样本，刷新率降低
 *               - 值越小：刷新率更高，实时性更好，但降低稳定性
 * 
 * @return 无
 * 
 * @note 
 * @warning 设置过高的采样点数可能导致刷新周期过长，影响基线的实时响应
 * @attention 
 * @see TKY_SetBaseDownRefreshSlow 相关正向刷新速率设置
 * @see TKY_SetBaseUpRefreshDouble 相关负向刷新速率设置
 * 
 * @example
 * // 设置为每采集1000个样本刷新一次基线
 * TKY_SetBaseRefreshSampleNum(1000);
 */
extern void TKY_SetBaseRefreshSampleNum(uint32_t newValue);

/**
 * @brief 设置基线向上刷新倍速参数
 * 
 * @param newValue 新的倍速模式设置值
 *                - 0：禁用倍速模式，使用正常刷新速率
 *                - 非0：启用倍速模式，值越大刷新速率越快
 * 
 * @return 无
 * 
 * @note 启用倍速模式会加快基线基线正向更新速率
 * @warning 
 * @attention 
 * 
 * @example
 */
extern void TKY_SetBaseUpRefreshDouble(uint8_t newValue);

/**
 * @brief 设置基线下行刷新慢速模式
 * 
 * @param newValue 慢速模式设置值
 *                - 0：禁用慢速模式，使用正常刷新速率
 *                - 非0：启用慢速模式，值越大刷新间隔越长
 * 
 * @return 无
 * 
 * @note 启用慢速模式会降低基线基线负向更新速率
 * @warning 
 * @attention 
 * 
 * @example
 */
extern void TKY_SetBaseDownRefreshSlow(uint8_t newValue);

/**
 * @brief 设置当前工作的滤波器模式
 * 
 * 该函数用于动态切换系统的过滤模式，改变信号处理算法的工作方式。
 * 适用于不同工况下的滤波需求调整。
 * 
 * @param newValue 滤波模式值
 *                - 具体模式定义参考系统过滤模式枚举或宏定义
 *                - 不同值对应不同的滤波算法或扫描方式
 * 
 * @return 无
 * 
 * @note 模式切换可能导致短暂的信号处理中断
 * @warning 切换过滤模式前需要先保存当前状态且清除历史数据
 * @see TKY_ClearHistoryData 相关历史数据清理函数
 * 
 * @example
 */
extern void TKY_SetFilterMode(uint8_t newValue);

/**
 * @brief 清除历史数据
 * 
 * 该函数根据当前过滤模式清除相应的历史数据缓冲区，用于：
 * - 系统初始化时的数据清零
 * - 过滤模式切换时的数据清理
 * - 手动触发历史数据重置
 * 
 * @param curFilterMode 当前过滤模式
 * 
 * @return 无
 * 
 * @note 清除操作不可逆，请确保在需要时调用
 * @warning 清除历史数据可能影响基于历史数据的算法精度，请谨慎使用
 * @attention 
 * 
 * @example
 */
extern void TKY_ClearHistoryData(uint8_t curFilterMode);


/**
 * @brief 保存触摸配置寄存器到库内部
 * 
 * @param 无
 * @return 无
 * 
 * @note 修改配置后立即保存
 * @warning 
 * @attention 
 * 
 * @example
 */
extern void TKY_SaveCfgReg(void);


/**
 * @brief 保存触摸寄存器配置并停止触摸扫描
 * 
 * @param 无
 * @return 无
 * 
 * @note 	若为阻塞扫描函数，则此函数仅保存touch寄存器配置
 *			若为非阻塞扫描函数，则此函数执行后保存touch寄存器配置且强行置内部状态机为空闲态
 * @warning 确保在调用前已完成所有关键数据的保存准备
 * @attention 此操作可能不可逆，请谨慎调用
 * 
 * @example
 */
extern void TKY_SaveAndStop(void);

/**
 * @brief 重新加载触摸寄存器配置并恢复touch扫描
 * 
 * @param 无
 * @return 无
 * 
 * @note 若为阻塞扫描函数，则此函数仅重新加载touch寄存器配置
 * 		 若为非阻塞扫描函数，则此函数执行后重新加载touch寄存器配置且恢复内部状态机为就绪态
 * @warning 调用此函数前应确保所有必要的初始化工作已完成
 * @attention 此函数涉及touch扫描状态的重置，请谨慎使用
 * 
 * @example
 */
extern void TKY_LoadAndRun(void);


/**
 * @brief 设置队列噪声阈值
 * 
 * 该函数用于设置指定队列的噪声检测阈值，用于判断信号中的噪声水平。
 * 当信号噪声超过设定阈值时，可触发相应处理。
 * 
 * @param groupidx     	  当前参数组索引（0~3）
 * @param noise_threshold 输入噪声样本，按顺序与触摸队列序列对应
 * @param dataNum		  噪声样本数量，应与最大通道数量一致
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置噪声阈值
 *                - 1：队列编号无超出范围
 * 
 * @note 噪声阈值设置影响系统的噪声检测灵敏度
 * @warning 阈值设置过低可能导致误报警，设置过高可能漏检真实噪声
 * @see TKY_NoiseEstimate 相关的噪声估计函数
 * 
 * @example
 * // 设置队列0的噪声阈值为30

 * uint8_t result = TKY_SetNoiseThreshold(0, 30);
 * if (result == 0) {
 *     // 阈值设置成功
 * }
 */
extern uint8_t TKY_SetCurGroupNoiseThreshold(uint8_t groupidx, uint16_t *noise_threshold, uint8_t dataNum);

/**
 * @brief 获取指定队列噪声阈值
 * 
 * 该函数用于读取指定队列的当前噪声阈值设置，用于状态监测、参数验证
 * 或动态调整前的阈值获取。噪声阈值用于判断信号中的噪声水平。
 * 
 * @param curQueueNum    指定队列编号
 *                     - 指定要获取噪声阈值的队列索引（0起始）
 * @param noise_threshold 输出参数，指向存储噪声阈值的缓冲区
 *                     - 返回当前设置的噪声阈值
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取噪声阈值
 *                - 1：队列编号超出范围
 * 
 * @note 获取的噪声阈值应与之前设置的阈值一致，除非被其他操作修改
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * @see TKY_SetCurQueueNoiseThreshold 相关的噪声阈值设置函数
 * 
 * @example
 * uint16_t currentNoiseThreshold;
 * uint8_t result = TKY_GetCurQueueNoiseThreshold(0, &currentNoiseThreshold);
 * if (result == 0) {
 *     // 成功获取噪声阈值
 *     printf("Queue 0 noise threshold: %u\n", currentNoiseThreshold);
 * }
 */
extern uint8_t TKY_GetCurQueueNoiseThreshold(uint8_t curQueueNum, uint16_t* noise_threshold);

/**
 * @brief 设置当前队列灵敏度级别
 * 
 * 该函数用于配置指定队列的灵敏度级别，调整按键触发敏感度。
 * 灵敏度级别影响队列对输入信号的响应特性，适用于不同应用场景下的灵敏度需求。
 * 该函数设置的值不一定为最终生效的灵敏度，需优先满足噪声评估结果。
 * 
 * @param curQueueNum 当前队列编号
 *                   - 指定要设置灵敏度的队列索引（0起始）
 * @param level       灵敏度级别：1~10，为0表示灵敏度由库内部自行决定
 *                   - 值越小通常表示灵敏度越高
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功设置灵敏度级别
 *                - 1：队列编号超出范围
 *				  - 2：设置灵敏度失败，噪声过大
 * 
 * @note 灵敏度级别设置影响信号检测的敏感度和抗干扰能力
 * @warning 过高的灵敏度可能导致误触发，过低的灵敏度可能漏检按键
 * @see TKY_SetCurQueueThreshold 相关的队列阈值设置函数
 * 
 * @example
 * // 设置队列0的灵敏度级别为4（高灵敏度）
 * uint8_t result = TKY_SetCurQueueSensitivityLevel(0, 4);
 * if (result == 0) {
 *     // 灵敏度设置成功
 * }
 */
extern uint8_t TKY_SetCurQueueSensitivityLevel(uint8_t curQueueNum, uint8_t level);


/**
 * @brief 获取指定队列灵敏度级别
 * 
 * 该函数用于读取指定队列的当前灵敏度级别设置，通过指针参数返回级别值。
 * 适用于需要获取灵敏度级别并进行状态监测、参数验证或动态调整的场景。
 * 
 * @param curQueueNum 指定队列编号
 *                   - 指定要获取灵敏度级别的队列索引（0起始）
 * @param level       输出参数，指向存储灵敏度级别的缓冲区
 *                   - 返回当前设置的灵敏度级别值
 *                   - 值越小表示灵敏度越高
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功获取灵敏度级别
 *                - 1：队列编号超出范围
 * 
 * @note 获取的灵敏度级别应与之前设置的级别一致，除非被其他操作修改(如噪声评估)
 * @warning 确保输出参数指针有效，否则可能导致内存访问错误
 * @see TKY_SetCurQueueSensitivityLevel 相关的灵敏度级别设置函数
 * 
 * @example
 * uint8_t currentLevel;
 * uint8_t result = TKY_GetCurQueueSensitivityLevel(0, &currentLevel);
 * if (result == 0) {
 *     // 成功获取灵敏度级别
 *     printf("Queue 0 sensitivity level: %u\n", currentLevel);
 * }
 */
extern uint8_t TKY_GetCurQueueSensitivityLevel(uint8_t curQueueNum, uint8_t* level);

/**
 * @brief 设置当前干扰通道数量
 * 
 * 该函数用于设置当前干扰通道数量，库内部检测到同时超过触发阈值的通道数量超过此值，
 * 则认为环境发生了变化或者环境中中有干扰存在。库内部会执行清楚触摸数据，重置基线的操作。
 * 
 * @param count 	  干扰通道数量
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：设置成功
 *                - 1：设置数量超出范围
 * 
 * @note 
 * @warning 
 * @see TKY_GetMaxNoiseChCount 相关的干扰通道数量获取函数
 * 
 */
extern uint8_t TKY_SetMaxNoiseChCount( uint8_t count );

/**
 * @brief 获取当前干扰通道数量
 * 
 * 该函数用于获取当前干扰通道数量，库内部检测到同时超过触发阈值的通道数量超过此值，
 * 则认为环境发生了变化或者环境中中有干扰存在。库内部会执行清楚触摸数据，重置基线的操作。
 * 
 * @param count 	 输出参数，指向存储干扰通道数量的缓冲区
 * 
 * @return uint8_t 返回操作状态码
 *                - 0：成功
 *                - 1：失败
 * 
 * @note 
 * @warning 
 * @see TKY_SetMaxNoiseChCount 相关的干扰通道数量设置函数
 * 
 */
extern uint8_t TKY_GetMaxNoiseChCount( uint8_t* count );

/**
 * @brief 使能触摸模块
 * 
 * @param 无
 * 
 * @return 无
 * 
 * @note 
 * @warning 
 * @see TKY_Disable 相关的关闭触摸模块函数
 * 
 */
extern void TKY_Enable(void);

/**
 * @brief 关闭触摸模块
 * 
 * @param 无
 * 
 * @return 无
 * 
 * @note 
 * @warning 
 * @see TKY_Enable 相关的使能触摸模块函数
 * 
 */
extern void TKY_Disable(void);


#endif
