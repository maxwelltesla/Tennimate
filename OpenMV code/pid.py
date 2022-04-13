from pyb import millis                             //pyb 模块包含了和 pyboard 相关的函数
from math import pi, isnan                         //用于检查给定数字是否为“ NaN” (不是数字)，它接受一个数字，如果给定数字为“ NaN” ，则返回True ，否则返回False 
//把算法抽象写成类，类中有类变量、成员变量、成员函数（类对象方法）
class PID:   //定义类
    _kp = _ki = _kd = _integrator = _imax = 0      //定义类变量
    _last_error = _last_derivative = _last_t = 0   //定义类变量
    _RC = 1/(2 * pi * 20)                          //定义类变量
    def __init__(self, p=0, i=0, d=0, imax=0):     //初始化成员变量
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float('nan')       //'nan'空的使用
 
    def get_pid(self, error, scaler):              //定义成员函数（不是类函数）
        tnow = millis()                            //当前时刻时间量
        dt = tnow - self._last_t                   //到当下的变化时间量
        output = 0                                 //输出初始值
        if self._last_t == 0 or dt > 1000:         //时间跨度为零：刚开始。时间跨度>1000，累计时长
            dt = 0                                 //到当下的变化时间量 置0
            self.reset_I()                         //重置积分项
        self._last_t = tnow                        //到当下的累计最终时间量
        delta_time = float(dt) / float(1000)       //delta_time  ：秒  
        output += error * self._kp                 //按比例值累计误差
        if abs(self._kd) > 0 and dt > 0:           //有时差和累计微分项   _kd，实际使用时多少个PID对象呢？单个PID对象，多次调用get_pid对象方法
            if isnan(self._last_derivative):       //_last_derivative为空
                derivative = 0
                self._last_derivative = 0
            else:                                  //_last_derivative非空
                derivative = (error - self._last_error) / delta_time        
            derivative = self._last_derivative + \                 //离散数字式PID和模拟PID的区别和相应的公式
                                     ((delta_time / (self._RC + delta_time)) * \
                                        (derivative - self._last_derivative))
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler
        if abs(self._ki) > 0 and dt > 0:            //有时差和累计积分项
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax: self._integrator = -self._imax
            elif self._integrator > self._imax: self._integrator = self._imax
            output += self._integrator
        return output
    def reset_I(self):                              //定义成员函数（不是类函数）
        self._integrator = 0
        self._last_derivative = float('nan')
