package fangzhen;  //erweifangzhen

import java.util.*;

public class main {
    public static void main(String[] args){
        q_node q_init = new q_node(0,0,null);
        q_node q_target= new q_node(100,100);
        double minPerStep = 3;
        double max_turn = 0.5;
        rode r = new rode(q_init,q_target,minPerStep,max_turn);
        Random random = new Random();
        double[] x = new double[]{10,30,30,40,55,60,80};
        double[] y = new double[]{10,20,60,45,55,20,85};
        obstacles os = new obstacles(x,y);
        while (!r.isEnd(r.l.get(r.l.size()-1))){  //判断的最后一个点为q_new,可能有错,
               //一直随机取点

            q_node q_rand = new q_node();
            int num = random.nextInt(101);
            if (num<=30){
                q_rand = r.q_target;   //这种赋值可以吗
            }
            else {  //随机取点
                double x1 = Math.random()*100; //0-100的伪随机数
                double y1 = Math.random()*100;
                q_rand.x = (x1);
                q_rand.y = (y1);
//                while (!q_rand.isLeagal(os)){  //如果点在障碍物里面,在障碍物里面也可以
//                    x1 = random.nextDouble();
//                    y1 = random.nextDouble();
//                    q_rand.x = (x1);
//                    q_rand.y = (y1);
//                }
            }
            System.out.println(q_rand.x+" "+q_rand.y);


            List<q_node> newAndNearRoot = r.findNew(q_rand);
            q_node q_new = newAndNearRoot.get(0);
            q_node q_nearRoot = newAndNearRoot.get(1);  //如果near是起点，那么nearroot就为空，就不用判断角度
            q_node q_near = newAndNearRoot.get(2);
            if(r.isAdd(q_new,q_nearRoot,q_near,os)){  //如果新点满足yuesu
//                System.out.println(q_new.x+" "+q_new.y);
                r.l.add(q_new);
                q_new.father = q_near;
                q_near.child.add(q_new);
            }
        }
        q_target.father = r.l.get(r.l.size()-1);
        q_node q = q_target;
        System.out.println(q.x+" "+q.y);
        double sum = 0;
        while (q.father!=null){
            System.out.println(q.father.x+" "+q.father.y);
            sum+= r.dis(q,q.father);
            q = q.father;
        }
        System.out.println(sum);
        System.out.println("所有的航迹点");
        for (q_node n: r.l){
            System.out.println(n.x+" "+ n.y);
        }
        //得到了一系列航迹点，将点化为图

    }

}
class rode{
    q_node q_init;
    q_node q_target;
    double minPerStep;
    List<q_node> l = new ArrayList<>(); //航迹
    double d;  //航迹距离约束
    double max_turn;  //60°对应的cos
    public rode(q_node q_init,q_node q_target,double minPerStep,double max_turn){
        this.minPerStep = minPerStep;
        this.q_target = q_target;
        this.q_init = q_init;
        this.max_turn = max_turn;
        this.l.add(q_init);
        d = 1.05*dis(q_init,q_target);
    }

    public List<q_node> findNew(q_node q_rand){ //寻找最优新点
        double min = 400;  //假设100x100
        q_node q_near = new q_node();
        q_node q_nearRoot = new q_node();
        q_node q_new = new q_node();
        List<q_node> newAndNearRoot = new ArrayList<>();
        for (int i=0;i<l.size();i++){
//            if(dis(l.get(i),q_rand)+dis(l.get(i),this.q_init)<min) {  //会不会最终都是起点
            if(dis(l.get(i),q_rand)<min) {  //会不会最终都是起点
                min = dis(l.get(i), q_rand);
                q_near = l.get(i);
                if (i!=0){
                    q_nearRoot = q_near.father;
                }
            }

        }
        //相似三角形计算移动的长度
        double delta_x = Math.abs(q_rand.x-q_near.x);
        double delta_y = Math.abs(q_rand.y-q_near.y);
        double x = this.minPerStep/dis(q_near,q_rand)*delta_x;
        double y = this.minPerStep/dis(q_near,q_rand)*delta_y;
        if(q_rand.x>q_near.x){
            q_new.x = q_near.x+x;
        }
        else {
            q_new.x = (q_near.x-x);
        }
        if(q_rand.y>q_near.y){
            q_new.y=(q_near.y+y);
        }
        else {
            q_new.y=(q_near.y-y);
        }
        newAndNearRoot.add(q_new);
        newAndNearRoot.add(q_nearRoot);
        newAndNearRoot.add(q_near);
        return newAndNearRoot;
    }

    public Boolean isAdd(q_node q_new,q_node q_near_root,q_node q_near,obstacles o){//判断新点是否满足约束

        if (dis(q_new,this.q_target)+hangJiCalculate(q_near)>=this.d){  //看航迹长度是否满足
            return false;
        }
        double x1 = q_near.x-q_near_root.x;  //这个向量的方向，正负这些可能会错！！！
        double x2 = q_new.x-q_near.x;
        double y1 = q_near.y-q_near_root.y;
        double y2 = q_new.y-q_near.y;
        if((x1 * x2 + y1 * y2) / (Math.sqrt(x1 * x1 + y1 * y1) * Math.sqrt(x2 * x2 + y2 * y2))<this.max_turn){  //角度
            return false;
        }
        if (!q_new.isLeagal(o)){
            return false;
        }
        if(!isCross(o,q_new,q_near)){   //两点连线之间是否穿过了障碍物
            return false;
        }
        return true;
    }

    public boolean isCross(obstacles o,q_node q_new,q_node q_near){
        double a = q_new.y-q_near.y;
        double b = q_near.x - q_new.x;
        double c = q_new.x*q_near.y - q_near.x*q_new.y;  //直线一般式
        for (int i=0;i<7;i++){
            if(Math.abs(o.x[i]*a+o.y[i]*b+c)/Math.sqrt(a*a+b*b)>o.radius){  //距离是否大于半径
                continue;
            }
            else {
                double m = a*o.y[i]-b*o.x[i];
                double x = -(a*c+b*m)/(a*a+b*b);
                double y = (m*a-c*b)/(a*a+b*b);
                if(Math.min(q_near.x,q_new.x)<=x&&x<=Math.max(q_near.x,q_new.x)&&Math.min(q_near.y,q_new.y)<=y&&y<=Math.max(q_near.y,q_new.y)){
                    return false;   //是否在线段上，在就返回false
                }
            }
        }
        return true;
     }

    public double hangJiCalculate(q_node q_near){
        double hangJi = minPerStep;
        while (!this.q_init.equals(q_near)){   //是否要重写equals,!!!
            hangJi+=minPerStep;
            q_near = q_near.father;
        }
        return hangJi;
    }

    public boolean isEnd(q_node q_new){//判断是否继续向下寻路
        if (dis(this.q_target,q_new)<minPerStep){
            return true;
        }
        return false;
    }

    public static double dis(q_node q1,q_node q2){
        return Math.sqrt(Math.pow(q1.x-q2.x,2)+Math.pow(q1.y-q2.y,2));
    }
}
class q_node{
    double x;
    double y;
    q_node father;
    List<q_node> child = new ArrayList<>();
    public q_node(double x,double y){
        this.x = x;
        this.y = y;
    }
    public q_node(double x,double y,q_node father){
        this.x = x;
        this.y = y;
        this.father = father;
    }
    public q_node(){

    }
    public boolean isLeagal(obstacles os){  //障碍和边界条件
        if (this.x>100||this.x<0||this.y<0||this.y>100){ //边界外
            return false;
        }
        for (int i=0;i<7;i++){
            if (rode.dis(new q_node(os.x[i],os.y[i]),this)< os.radius){ //障碍物内部
                return false;
            }
        }

        return true;
    }


    public boolean equals(Object o) {
        if (this==o){
            return true;
        }
        q_node q = (q_node) o;
        return this.x==q.x&&this.y==q.y;//&&this.father.equals(q.father)&&this.child.equals(q.child);
    }
    public int hashCode(){    //可能有问题
        return Objects.hash(x,y);
    }
}
class obstacles{
    int radius = 8;
    double[] x = new double[7];
    double[] y = new double[7];
    public obstacles(double[] x,double[] y){
        for (int i=0;i<7;i++){
            this.x[i] = x[i];
            this.y[i] = y[i];
        }
    }
}