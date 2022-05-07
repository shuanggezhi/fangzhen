package fangzhen;

import java.util.*;

public class main {
    public static void main(String[] args){
        rode r = new rode();
        while (!r.isEnd(r.l.get(r.l.size()-1))){  //判断的最后一个点为q_new,可能有错,
               //一直随机取点
        }
    }

}
class rode{
    List<Double> q_init;
    List<Double> q_target;
    Double minPerStep;
    List<ArrayList<Double>> l = new ArrayList<>(); //航迹
    Double d;  //航迹距离约束
    double max_turn;  //60°对应的cos
    public rode(List<Double> q_init,List<Double> q_target,Double minPerStep,double max_turn){
        this.minPerStep = minPerStep;
        this.q_target = q_target;
        this.q_init = q_init;
        this.max_turn = max_turn;
        d = 1.5*Math.sqrt(Math.pow(q_target.get(0)-q_init.get(0),2)+Math.pow(q_target.get(1)-q_init.get(1),2));
    }

    public void findNew(List<Double> q_rand){ //寻找最优新点

    }
    public void isAdd(List<Double> q_new){//判断新点是否满足约束

    }
    public boolean isEnd(List<Double> q_new){//判断是否继续向下寻路
        if (Math.sqrt(Math.pow(this.q_target.get(0)-q_new.get(0),2)+Math.pow(this.q_target.get(1)-q_new.get(1),2))<minPerStep){
            return true;
        }
        return false;
    }
}