# Makefile for mpu.c

# 编译器
CC = gcc

# 编译选项
CFLAGS = -Wall -O2 -lwiringPi

# 目标文件
TARGET = mpu

# 源文件
SRCS = mpu.c

# 对象文件
OBJS = $(SRCS:.c=.o)

# 默认目标
all: $(TARGET)

# 生成目标文件
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(CFLAGS)

# 生成对象文件
%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

# 清理生成的文件
clean:
	rm -f $(OBJS) $(TARGET)

# 重新编译
rebuild: clean all
