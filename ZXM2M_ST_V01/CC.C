
/******************************************************************************
* Macros(下车通信): 起重机下车数据序号和地址定义
******************************************************************************/
//==TAG-A5E0节点状态==============================================================
#define ZXDOWN_A5E0_ADDR    0  // 起始地址
//==TAG-A5E1传动系统==============================================================
#define ZXDOWN_A5E1_ADDR    (ZXDOWN_A5E0_ADDR+SIZE_OF_ZXDOWN_A5E0)  // 起始地址
//==TAG-A5E2支腿相关信息==========================================================
#define ZXDOWN_A5E2_ADDR    (ZXDOWN_A5E1_ADDR+SIZE_OF_ZXDOWN_A5E1)  // 起始地址
//==TAG-A5E3悬挂系统==============================================================
#define ZXDOWN_A5E3_ADDR    (ZXDOWN_A5E2_ADDR+SIZE_OF_ZXDOWN_A5E2)  // 起始地址
//==TAG-A5E4转向系统（全地面）====================================================
#define ZXDOWN_A5E4_ADDR    (ZXDOWN_A5E3_ADDR+SIZE_OF_ZXDOWN_A5E3)  // 起始地址
//==TAG-A5E5转向系统（汽车）======================================================
#define ZXDOWN_A5E5_ADDR    (ZXDOWN_A5E4_ADDR+SIZE_OF_ZXDOWN_A5E4)  // 起始地址
//==TAG-A5E6制动系统==============================================================
#define ZXDOWN_A5E6_ADDR    (ZXDOWN_A5E5_ADDR+SIZE_OF_ZXDOWN_A5E5)  // 起始地址
//==TAG-A5E7动力系统==============================================================
#define ZXDOWN_A5E7_ADDR    (ZXDOWN_A5E6_ADDR+SIZE_OF_ZXDOWN_A5E6)  // 起始地址
//==TAG-A5E8单发取力系统==========================================================
#define ZXDOWN_A5E8_ADDR    (ZXDOWN_A5E7_ADDR+SIZE_OF_ZXDOWN_A5E7)  // 起始地址
//==TAG-A5E9液压系统==============================================================
#define ZXDOWN_A5E9_ADDR    (ZXDOWN_A5E8_ADDR+SIZE_OF_ZXDOWN_A5E8)  // 起始地址
//==TAG-A5EA双动力驱动系统========================================================
#define ZXDOWN_A5EA_ADDR    (ZXDOWN_A5E9_ADDR+SIZE_OF_ZXDOWN_A5E9)  // 起始地址
//==TAG-A5EB轮胎胎压==============================================================
#define ZXDOWN_A5EB_ADDR    (ZXDOWN_A5EA_ADDR+SIZE_OF_ZXDOWN_A5EA)  // 起始地址
//==TAG-A5EC左支腿面板============================================================
#define ZXDOWN_A5EC_ADDR    (ZXDOWN_A5EB_ADDR+SIZE_OF_ZXDOWN_A5EB)  // 起始地址
//==TAG-A5ED右支腿面板============================================================
#define ZXDOWN_A5ED_ADDR    (ZXDOWN_A5EC_ADDR+SIZE_OF_ZXDOWN_A5EC)  // 起始地址
//==TAG-A5EE中控台输入信息========================================================
#define ZXDOWN_A5EE_ADDR    (ZXDOWN_A5ED_ADDR+SIZE_OF_ZXDOWN_A5ED)  // 起始地址
//==TAG-A5A3支腿作业信息==========================================================
#define ZXDOWN_A5A3_ADDR    (ZXDOWN_A5EE_ADDR+SIZE_OF_ZXDOWN_A5EE)  // 起始地址
//==TAG-A5A4辅助支腿作业信息=======================================================
#define ZXDOWN_A5A4_ADDR    (ZXDOWN_A5A3_ADDR+SIZE_OF_ZXDOWN_A5A3)  // 起始地址


