import random
import tkinter
 
root = tkinter.Tk()
root.title('飞机大战')
root.geometry('800x650+400+100')
canvas = tkinter.Canvas(root, width=550, height=520, bg='white')
canvas.pack()
lb = tkinter.Label(root, width=20, height=1, bg='white')
lb.place(x=290, y=540)
count = 0
message = "消灭敌机数量："
lb['text'] = message + str(count)
lb2 = tkinter.Label(root, width=20, height=1, bg='white')
lb2.place(x=290, y=570)
lb2['text'] = '---'
 
 
# 玩家
class PlayerPlane:
    # 机翼的坐标
    wings_x1, wings_y1 = 240, 470
    wings_x2, wings_y2 = 290, 470
    # 尾翼的坐标
    tail_x1, tail_y1 = 250, 500
    tail_x2, tail_y2 = 280, 500
    # 机身的坐标
    fuselage_x1, fuselage_y1 = 265, 500
    fuselage_x2, fuselage_y2 = 265, 440
    # 子弹的坐标
    bullet_x1, bullet_y1 = fuselage_x2, fuselage_y2
    bullet_x2, bullet_y2 = bullet_x1, bullet_y1 - 120
 
    def __init__(self):
        # 机翼
        self.wings = canvas.create_line(self.wings_x1, self.wings_y1, self.wings_x2, self.wings_y2)
        # 尾翼
        self.tail = canvas.create_line(self.tail_x1, self.tail_y1, self.tail_x2, self.tail_y2)
        # 机身
        self.fuselage = canvas.create_line(self.fuselage_x1, self.fuselage_y1, self.fuselage_x2, self.fuselage_y2)
        # 子弹
        self.bullet = None
        # 移动距离
        self.move_distance = 5
 
    # 攻击
    def attack(self, event):
        # 子弹的坐标
        self.bullet_x1, self.bullet_y1 = self.fuselage_x2, self.fuselage_y2
        self.bullet_x2, self.bullet_y2 = self.bullet_x1, self.bullet_y1 - 120
        # 子弹
        self.bullet = canvas.create_line(self.bullet_x1, self.bullet_y1, self.bullet_x2, self.bullet_y2, fill='red',
                                         dash=(1, 1))
        # 80毫秒之后子弹消失
        canvas.after(80, self.delete_bullet)
 
    # 删除子弹
    def delete_bullet(self):
        canvas.delete(self.bullet)
 
    # 左移
    def move_left(self, event):
        if self.wings_x1 == 0:
            return
        canvas.move(self.wings, -self.move_distance, 0)
        self.wings_x1 -= self.move_distance
        self.wings_x2 -= self.move_distance
 
        canvas.move(self.tail, -self.move_distance, 0)
        self.tail_x1 -= self.move_distance
        self.tail_x2 -= self.move_distance
 
        canvas.move(self.fuselage, -self.move_distance, 0)
        self.fuselage_x1 -= self.move_distance
        self.fuselage_x2 -= self.move_distance
 
    # 右移
    def move_right(self, event):
        if self.wings_x2 == 550:
            return
        canvas.move(self.wings, self.move_distance, 0)
        self.wings_x1 += self.move_distance
        self.wings_x2 += self.move_distance
 
        canvas.move(self.tail, self.move_distance, 0)
        self.tail_x1 += self.move_distance
        self.tail_x2 += self.move_distance
 
        canvas.move(self.fuselage, self.move_distance, 0)
        self.fuselage_x1 += self.move_distance
        self.fuselage_x2 += self.move_distance
 
    # 上移
    def move_up(self, event):
        if self.fuselage_y2 == 0:
            return
        canvas.move(self.wings, 0, -self.move_distance)
        self.wings_y1 -= self.move_distance
        self.wings_y2 -= self.move_distance
 
        canvas.move(self.tail, 0, -self.move_distance)
        self.tail_y1 -= self.move_distance
        self.tail_y2 -= self.move_distance
 
        canvas.move(self.fuselage, 0, -self.move_distance)
        self.fuselage_y1 -= self.move_distance
        self.fuselage_y2 -= self.move_distance
 
    # 下移
    def move_down(self, event):
        if self.fuselage_y1 == 530:
            return
        canvas.move(self.wings, 0, self.move_distance)
        self.wings_y1 += self.move_distance
        self.wings_y2 += self.move_distance
 
        canvas.move(self.tail, 0, self.move_distance)
        self.tail_y1 += self.move_distance
        self.tail_y2 += self.move_distance
 
        canvas.move(self.fuselage, 0, self.move_distance)
        self.fuselage_y1 += self.move_distance
        self.fuselage_y2 += self.move_distance
 
 
# 敌机类
class EnemyPlane:
    wings_x1 = 0
    wings_y1 = 30
    wings_x2 = 40
    wings_y2 = 30
    tail_x1 = 10
    tail_y1 = 0
    tail_x2 = 30
    tail_y2 = 0
    fuselage_x1 = 20
    fuselage_y1 = 0
    fuselage_x2 = 20
    fuselage_y2 = 60
    # 子弹的坐标
    bullet_x1, bullet_y1 = fuselage_x2, fuselage_y2
    bullet_x2, bullet_y2 = bullet_x1, bullet_y1 + 120
 
    def __init__(self):
        self.bullet = None
        # 移动距离
        self.move_distance = 10
 
    def create(self):
        lt = [0, 40, 80, 120]
        x_change = random.choice(lt)
        self.wings_x1 += x_change
        self.wings_x2 += x_change
        self.tail_x1 += x_change
        self.tail_x2 += x_change
        self.fuselage_x1 += x_change
        self.fuselage_x2 += x_change
        # 机翼
        self.wings = canvas.create_line(self.wings_x1, self.wings_y1, self.wings_x2, self.wings_y2)
        # 尾翼
        self.tail = canvas.create_line(self.tail_x1, self.tail_y1, self.tail_x2, self.tail_y2)
        # 机身
        self.fuselage = canvas.create_line(self.fuselage_x1, self.fuselage_y1, self.fuselage_x2, self.fuselage_y2)
        self.move()
 
    def attack(self):
        # 子弹的坐标
        self.bullet_x1, self.bullet_y1 = self.fuselage_x2, self.fuselage_y2
        self.bullet_x2, self.bullet_y2 = self.bullet_x1, self.bullet_y1 + 120
        # 子弹
        self.bullet = canvas.create_line(self.bullet_x1, self.bullet_y1, self.bullet_x2, self.bullet_y2, fill='red',
                                         dash=(1, 1))
        # 80毫秒之后子弹消失
        canvas.after(100, self.delete_bullet)
 
    # 删除子弹
    def delete_bullet(self):
        canvas.delete(self.bullet)
 
    def delete(self):
        canvas.delete(self.wings)
        canvas.delete(self.tail)
        canvas.delete(self.fuselage)
 
    def move(self):
        global count
        self.attack()
 
        canvas.move(self.wings, 0, self.move_distance)
        self.wings_y1 += self.move_distance
        self.wings_y2 += self.move_distance
 
        canvas.move(self.tail, 0, self.move_distance)
        self.tail_y1 += self.move_distance
        self.tail_y2 += self.move_distance
 
        canvas.move(self.fuselage, 0, self.move_distance)
        self.fuselage_y1 += self.move_distance
        self.fuselage_y2 += self.move_distance
        time_id2 = root.after(2000, self.move)
 
        if self.fuselage_y2 == 530:
            root.after_cancel(time_id2)
            self.delete()
 
        # 检测是否受到攻击
        if (self.wings_x1 <= p.bullet_x2 <= self.wings_x2 and self.wings_y2 >= p.bullet_y2) or (
                p.bullet_x2 == self.fuselage_x2 and p.bullet_y2 <= self.fuselage_y2):
            root.after_cancel(time_id2)
            self.delete()
            count += 1
            lb['text'] = message + str(count)
            if count == n:
                lb2['text'] = '敌机已全部消灭！'
 
# 创建玩家飞机
p = PlayerPlane()
#  存放敌机
lt = []
# 敌机数量
n = 3
for i in range(n):
    lt.append(EnemyPlane())
i = 0
 
def cre():
    global i, n, count
    time_id = root.after(6000, cre)
    if i < n:
        lt[i].create()
        i += 1
    else:
        root.after_cancel(time_id)
# 创建敌机
cre()
 
# 绑定事件
canvas.bind('<KeyPress-j>', p.attack)
canvas.bind('<KeyPress-a>', p.move_left)
canvas.bind('<KeyPress-d>', p.move_right)
canvas.bind('<KeyPress-w>', p.move_up)
canvas.bind('<KeyPress-s>', p.move_down)
# 设置按钮获取焦点
canvas.focus_set()
 
root.mainloop()
