#****************************************************************
# @FileName: Makefile.env
# @Engineer: TenYan
# @version   V1.0
# @Date:     2020-12-24
# 用于编译单文件夹,多文件
# $@ 表示规则中的目标文件集
# $^ 表示所有的依赖目标的集合
# $< 表示依赖目标中的第一个目标名字
#***************************************************************
#编译工具链
CROSS-COMPILE:=arm-none-linux-gnueabi-
CC = $(CROSS-COMPILE)gcc
LD = $(CROSS-COMPILE)ld

#编译选项
CFLAGS = -s -Wall -I${DIR_INC}
LDFLAGS = -ldl -lm -lpthread

#定义输出文件名
TARGET = $(DIR_BIN)/helloworld

#定义根目录
DIR_INC = ./inc
DIR_OBJ = ./obj
DIR_BIN = ./bin
DIR_SRC = ./src

SRC = $(wildcard ${DIR_SRC}/*.c)
DIR = $(notdir $(SRC))

#把所有的.c替换为.o
OBJ = $(patsubst %.c,${DIR_OBJ}/%.o,$(notdir ${SRC}))

all: $(TARGET)

#生成.o的规则
$(TARGET): $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

${DIR_OBJ}/%.o: ${DIR_SRC}/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -rf $(OBJ) $(TARGET)

