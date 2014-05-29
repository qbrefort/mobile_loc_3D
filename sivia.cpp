#include "sivia.h"
#include <math.h>

#include <stdlib.h>


void Sivia::contract_and_draw(Ctc& c, IntervalVector& X,IntervalVector& viinside,int isctcinside,struct sivia_struct *my_struct,int& nbox, const QColor & pencolor, const QColor & brushcolor) {
    IntervalVector X0= X;       // get a copy
    try {
        c.contract(X);
        if (X==X0) return;     // nothing contracted.
        IntervalVector* rest;
        int n=X0.diff(X,rest); // calculate the set difference
        for (int i=0; i<n; i++) {     // display the boxes
            R.DrawBox(rest[i][0].lb(),rest[i][0].ub(), rest[i][1].lb(),rest[i][1].ub(),QPen(pencolor),QBrush(brushcolor));
            if (isctcinside==1) {
                viinside = rest[i];
                my_struct->vin.push_back(viinside);
                nbox++;
                my_struct->isinside=1;
                my_struct->areain += viinside[0].diam()*viinside[1].diam();
                //cout<<viinside<<endl;
            }
        }
        delete[] rest;
    } catch(EmptyBoxException&) {
        R.DrawBox(X0[0].lb(),X0[0].ub(),X0[1].lb(),X0[1].ub(),QPen(pencolor),QBrush(brushcolor));
    }
}

Sivia::Sivia(repere& R,struct sivia_struct *my_struct) : R(R) {

    my_struct->areain = 0;
    my_struct->areap = 0;
    my_struct->isinside=0;
    Variable xvar,yvar,zvar,tvar;
    int n = my_struct->nb_beacon;
    double *x=my_struct->x; // vecteur des abcisses des donnees
    double *y=my_struct->y; // vecteur des ordonnees des donnees
    double *z=my_struct->z;
    double *r=new double[n]; // vecteur des rayons
    //r1=9.0;r2=2.0;r3=6.0;r4=6.0;r5=10.0;
    //random config (original config)
    //y1=x1=14;x2=4;y2=-7;x3=7;y3=10;x4=y4=-10;x5=-4;y5=12;x6=0;y6=0;
    //square config
//    x1=-24;y1=24;x2=-24;y2=-24;x3=24;y3=24;x4=24;y4=-24;x5=y5=0;x6=y6=0;

    //Create a square pattern of beacon with the 5th in the center, after 6 beacons, their position is generated at random
//    if (n>=2){
//        x[0]=-24;y[0]=24;x[1]=-24;y[1]=22;
//    }
//    if (n>=3){
//        x[2]=-24;y[2]=-24;
//    }
//    if (n>=4){
//        x[3]=24;y[3]=24;
//    }
//    if (n>=5){
//        x[4]=24;y[4]=-24;
//    }
//    if (n>=6){
//        x[5]=y[5]=0;
//    }

    double xr=my_struct->robot_position[0],yr=my_struct->robot_position[1],zr=my_struct->robot_position[2];

    for (int i=0;i<n;i++) {
//        r[i]= sqrt(pow(xr-x[i],2)+pow(yr-y[i],2)+pow(zr-z[i],2));
        r[i]= sqrt(pow(xr-x[i],2)+pow(yr-y[i],2));
        if (my_struct->outliers[i]!=0)
            r[i] *= (1+my_struct->outliers[i]*my_struct->erroutlier/100);
    }

    vector<Function*> f;
    vector<Function*> fp;
    vector<Function*> theta_1;
    vector<Function*> theta_2;
    double th1[n];
    double th2[n];
    for (int i=0;i<n;i++){
        th1[i] = my_struct->theta_sonar[i];
        th2[i] = th1[i] + 20;
    }

    for(int i=0;i<n;i++) {
//        f.push_back(new Function(xvar,yvar,zvar,sqrt(sqr(xvar-x[i])+sqr(yvar-y[i])+sqr(zvar-z[i]))));
        f.push_back(new Function(xvar,yvar,sqrt(sqr(xvar-Interval(x[i]-my_struct->beacon_interval*r[i]/100,x[i]+my_struct->beacon_interval*r[i]/100))
                                                +sqr(yvar-Interval(y[i]-my_struct->beacon_interval*r[i]/100,y[i]+my_struct->beacon_interval*r[i]/100)))));
         fp.push_back(new Function(xvar,yvar,tvar,sqrt(sqr(xvar-x[i])+sqr(yvar-y[i]))));
         theta_1.push_back(new Function(xvar,yvar,yvar-y[i]-((r[i]*sin(th1[i])-y[i])/(r[i]*cos(th1[i])-x[i]))*(xvar-x[i])));
         theta_2.push_back(new Function(xvar,yvar,yvar-y[i]-((r[i]*sin(th2[i])-y[i])/(r[i]*cos(th2[i])-x[i]))*(xvar-x[i])));
    }

    vector<Ctc*> vec_out;
    vector<Ctc*> vec_in;
    CtcNotIn* intemp1,*intemp2,*intemp3;
    CtcIn* outtemp1,*outtemp2,*outtemp3;
//    for(int i=0;i<n;i++) {

//        if (cos(th1[i])>0){
//            outtemp1 = (new CtcIn(*(theta_1[i]),Interval(0,100)));
//            intemp1 = (new CtcNotIn(*(theta_1[i]),Interval(0,100)));
//        }
//        else{
//            outtemp1 = (new CtcIn(*(theta_1[i]),Interval(-100,0)));
//            intemp1 = (new CtcNotIn(*(theta_1[i]),Interval(-100,0)));
//        }
//        if (cos(th2[i])>0){
//            outtemp2 = (new CtcIn(*(theta_2[i]),Interval(-100,0)));
//            intemp2 =(new CtcNotIn(*(theta_2[i]),Interval(-100,0)));
//        }
//        else{
//            outtemp2 = (new CtcIn(*(theta_2[i]),Interval(0,100)));
//            intemp2 =(new CtcNotIn(*(theta_2[i]),Interval(0,100)));
//        }
//        outtemp3 = (new CtcIn(*(f[i]),(r[i]+Interval(-1,1)*my_struct->err[i])));
//        intemp3 = (new CtcNotIn(*(f[i]),(r[i]+Interval(-1,1)*my_struct->err[i])));

//        vec_out.push_back(new CtcUnion(*outtemp1,*outtemp2,*outtemp3));
//        vec_in.push_back(new CtcCompo(*intemp1,*intemp2,*intemp3));
//    }
//    free(intemp1);free(intemp2);free(intemp3);free(outtemp1);free(outtemp2);free(outtemp3);
//    cout<<"coucou"<<endl;
    for(int i=0;i<n;i++) {
        vec_out.push_back(new CtcIn(*(f[i]),(r[i]+Interval(-1,1)*my_struct->err[i])));
        vec_in.push_back(new CtcNotIn(*(f[i]),(r[i]+Interval(-1,1)*my_struct->err[i])));
    }

    double re=0.5;
    int maxq = my_struct->nb_beacon; //nb of contractors
    int ctcq = maxq - my_struct->q + 1; //nb for q-relaxed function of Ibex
    int ninbox = 0;

    CtcQInter insidetmp(vec_in,ctcq);
    CtcQInter outsidetmp(vec_out,my_struct->q);
    CtcFixPoint inside(insidetmp);
    CtcFixPoint outside(outsidetmp);
    IntervalVector box = my_struct->box.back();
//    IntervalVector box(3);box[0]=box[1]=Interval(-25,25);box[2]=Interval(0,2);
    IntervalVector viinside(3);
    //vin.resize(4);

    LargestFirst lf;
    stack<IntervalVector> s;
    s.push(box);
    my_struct->in_perhaps=0;
    while (!s.empty()) {
        IntervalVector box=s.top();
        s.pop();
        contract_and_draw(inside,box,viinside,1,my_struct,ninbox,Qt::magenta,Qt::red);
        if (box.is_empty()) { continue; }

        contract_and_draw(outside,box,viinside,0,my_struct,ninbox,Qt::darkBlue,Qt::cyan);
        if (box.is_empty()) { continue; }

        if (box.max_diam()<my_struct->epsilon_sivia) {
            R.DrawBox(box[0].lb(),box[0].ub(),box[1].lb(),box[1].ub(),QPen(Qt::yellow),QBrush(Qt::white));
            my_struct->areap += box[0].diam()*box[1].diam();
            my_struct->in_perhaps=1;
        } else {
            pair<IntervalVector,IntervalVector> boxes=lf.bisect(box);
            s.push(boxes.first);
            s.push(boxes.second);
        }
    }

    double tx[ninbox],ty[ninbox],tz[ninbox];

    //cout<<"next"<<ninbox<<endl;
    for(int i=0;i<ninbox;i++){
        IntervalVector cur = (my_struct->vin.back());
        my_struct->vin_prev.push_back(cur);
        //cout<<cur<<endl;
        Interval xcur=cur[0];
        Interval ycur=cur[1];
        Interval zcur=cur[2];
        tx[i]=xcur.mid();
        ty[i]=ycur.mid();
        tz[i]=zcur.mid();
        my_struct->vin.pop_back();
    }


    double xin=0,yin=0,zin=0;
    for(int i=0;i<ninbox;i++){
        xin += tx[i];
        yin += ty[i];
        zin += tz[i];
    }
    xin/=double(ninbox);
    yin/=double(ninbox);
    zin/=double(ninbox);

    my_struct->robot_position_found[0] = xin;
    my_struct->robot_position_found[1] = yin;
    my_struct->robot_position_found[2] = zin;
    for(int i=0;i<n;i++)
        R.DrawEllipse(x[i],y[i],re,QPen(Qt::black),QBrush(Qt::NoBrush));

    my_struct->vin.clear();
    vec_out.clear();
    vec_in.clear();
    f.clear();
}

