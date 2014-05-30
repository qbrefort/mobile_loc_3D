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
            if (isctcinside==1) {
                R.DrawBox(rest[i][0].lb(),rest[i][0].ub(), rest[i][1].lb(),rest[i][1].ub(),QPen(pencolor),QBrush(brushcolor));
                viinside = rest[i];
                my_struct->vin.push_back(viinside);
                nbox++;
                my_struct->isinside=1;
                my_struct->areain += viinside[0].diam()*viinside[1].diam()*viinside[2].diam();
                //cout<<viinside<<endl;
            }
        }
        delete[] rest;
    } catch(EmptyBoxException&) {
        if (isctcinside==1)
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
    double xr=my_struct->robot_position[0],yr=my_struct->robot_position[1],zr=my_struct->robot_position[2];

    for (int i=0;i<n;i++) {
//        r[i]= sqrt(pow(xr-x[i],2)+pow(yr-y[i],2)+pow(zr-z[i],2));
        r[i]= sqrt(pow(xr-x[i],2)+pow(yr-y[i],2)+pow(zr-z[i],2));
        if (my_struct->outliers[i]!=0)
            r[i] *= (1+my_struct->outliers[i]*my_struct->erroutlier/100);
    }

    vector<Function*> f;
    double th1[n];
    double th2[n];
    for (int i=0;i<n;i++){
        th1[i] = my_struct->theta_sonar[i];
        th2[i] = th1[i] + 20;
    }

    for(int i=0;i<n;i++) {
//        f.push_back(new Function(xvar,yvar,zvar,sqrt(sqr(xvar-x[i])+sqr(yvar-y[i])+sqr(zvar-z[i]))));
        f.push_back(new Function(xvar,yvar,zvar,sqrt(sqr(xvar-Interval(x[i]-my_struct->beacon_interval*r[i]/100,x[i]+my_struct->beacon_interval*r[i]/100))
                                                +sqr(yvar-Interval(y[i]-my_struct->beacon_interval*r[i]/100,y[i]+my_struct->beacon_interval*r[i]/100))
                                                +sqr(zvar-Interval(z[i]-my_struct->beacon_interval*r[i]/100,z[i]+my_struct->beacon_interval*r[i]/100)))));
    }

    vector<Ctc*> vec_out;
    vector<Ctc*> vec_in;

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
            //R.DrawBox(box[0].lb(),box[0].ub(),box[1].lb(),box[1].ub(),QPen(Qt::yellow),QBrush(Qt::white));
            my_struct->areap += box[0].diam()*box[1].diam()*box[2].diam();
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

