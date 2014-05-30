#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ibex.h"
#include "sivia.h"
#include <math.h>

#include <fstream>

#include <QElapsedTimer>
vector<double> u,xv,yv,zv,vv,thetav,xc,yc,errpos;
double t;
int timeinfo=1;
int drawapproxpos=0;
int drawarrow=0;
double xmin=-25,xmax=25,ymin=-25,ymax=25;
int nboutlier;
int probsensorfalse;
int step=1;
double cantlocalize;
bool first_simu = false;
repere *R;
sivia_struct *par = new sivia_struct(); // SIVIA parameters


double MainWindow::sign(double a){
    if (a >= 0)
        return 1;
    else
        return -1;
}

QColor MainWindow::traj_color(double i){
    double max = *(max_element(zv.begin(),zv.end()));
    double min = *(min_element(zv.begin(),zv.end()));
    double r=10,g=50,b=10;
    double coef = (0.25*zv[i]);
    if (coef!=0){
        r = r/coef+r;
        g = g/coef+g;
        b = b/coef+b;
    }
    if (g>255) g=255;
    //cout<<r<<" "<<g<<" "<<b<<endl;
    //cout<<coef<<endl;
    return QColor(r,g,b);

}

MainWindow::MainWindow(QWidget *parent) :  QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    GenTraj();
    par->x = new double[100];
    par->y = new double[100];
    par->z = new double[100];
    par->outliers = new int[100];
    par->err = new double[100];
    par->theta_sonar = new double[100];
    par->in_perhaps = 0;
    par->isinside = 0;
    par->nb_beacon = ui->BeaconSpinBox->value();;
    par->box.resize(1);
    par->beacon_interval = ui->BeaconPosSpinBox->value();
    ui->BeaconPosSpinBox->setValue(0);
    ui->BeaconSpinBox->setValue(5);
}

MainWindow::~MainWindow() {
    free(R);
    free(par->x);
    free(par->y);
    free(par->z);
    free(par->outliers);
    free(par->err);
    free(par);
    delete ui;

}

// Window Initialization
void MainWindow::Init() {
    R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    par->epsilon_sivia=ui->EpsilonSpinBox->value();
    par->q=ui->InterSpinBox->value();
    probsensorfalse=ui->probsensorisfalseSpinBox->value();
    par->erroutlier = ui->errorOutlierSpinBox->value();
    par->nb_beacon = ui->BeaconSpinBox->value();
    for (int i=0;i<100;i++) par->err[i]=0.2;
}

// Generates the trajectory of the robot. Called only once in initialization of simulation
void MainWindow::GenTraj(){
    // 8 curve
    int nb_step=6500;

    par->theta = new double[nb_step];
    par->speedx = new double[nb_step];
    par->speedy = new double[nb_step];
    xv.resize(nb_step); yv.resize(nb_step); zv.resize(nb_step);vv.resize(nb_step); thetav.resize(nb_step); u.resize(nb_step);
    xv[0] = 0; yv[0] = 0; zv[0] = 0; thetav[0]=0; vv[0]=0;
    double dt=0.02;
    double w=0.05;
    double tt=0;
    for(int i= 1; i < nb_step; i++){
        tt = tt+dt;
        u[i-1]= 0.1*sign(sin(w*tt));
        yv[i] = yv[i-1] + dt*sin(thetav[i-1]);
        xv[i] = xv[i-1] + dt*cos(thetav[i-1]);
        zv[i] =  2+cos(dt*i);
        vv[i] = sqrt(pow((xv[i]-xv[i-1])/dt,2)+pow((yv[i]-yv[i-1])/dt,2));
//        zv[i] = 0;
        thetav[i] = thetav[i-1] + dt*(u[i-1]);
        par->theta[i] = thetav[i];
        par->speedx[i] = (xv[i]-xv[i-1]);
        par->speedy[i] = (yv[i]-yv[i-1]);
    }
}

// Update the robot current position with time t
void MainWindow::RobotTraj(){
    par->robot_position[0]=xv[t];
    par->robot_position[1]=yv[t];
    par->robot_position[2]=zv[t];
    par->robot_position[3]=thetav[t];
}

// Launch a simulation given the method, creates a log file and an info window at the end
void MainWindow::Simu(int method){
    first_simu = true;
    double errgomne=0;
    int cpt=0,cpt2=0;
    cantlocalize=0;
    nboutlier = 0;
    errpos.clear();
    par->box.clear();
    par->box.push_back(IntervalVector(3,Interval(-25,25)));
    ui->checkBox->setChecked(false);
    for(int i=0;i<par->nb_beacon;i++){
        par->x[i]= 1*(25 - rand() % 50);
        par->y[i]= 1*(25 - rand() % 50);
        par->z[i]= 5-i%4;
        par->theta_sonar[i] = rand() % 360;
        if((rand() % 100) <= probsensorfalse){
            if (rand() % 1 ==1)
                par->outliers[i]=1;
            else par->outliers[i]=-1;
            nboutlier++;
        }
        else{par->outliers[i]=0;}
    }
    ui->nbOutlierlcd->display(nboutlier);
    par->nb_beacon = ui->BeaconSpinBox->value();
    //Log
    ofstream myfile;
    myfile.open ("log_simu.txt");
    remove("log_simu.txt");
    myfile.close();
    myfile.open ("log_simu.txt");
    QString vt = "";
    QElapsedTimer tsimu;
    int gomnecpt=0;
    tsimu.start();
    step=ui->step_SpinBox->value();
    for(double i=0;i<6500;i=i+step) cpt2++;
    par->ratio_area.clear();
    par->areain=0;
    par->areap=0;
    for(double i=0;i<6500;i=i+step){
        //cout<<"entry box :"<<par->box.back()<<endl;
        QElapsedTimer tcur;
        QString vtcur = "";
        tcur.start();
        t=i;
        par->iteration=t;
        if(method==1)   on_ButtonFindSol_clicked();
        if (method==2)  on_ButtonGOMNE_clicked();
        if(method==3) {
            int qtmp=par->q;
            on_ButtonGOMNE_clicked();
            SLAM(step);
            if (par->q==qtmp) gomnecpt++;
            //if (gomnecpt>4) method=4;
        }
        if(method==4) GOMNE_fixed_q();
        errgomne += fabs(double(par->q-(double(par->nb_beacon) -  double(nboutlier))) / (double(par->nb_beacon) -  double(nboutlier))*100);
        ui->TSlider->setValue(t);
        Zoom(step);
        delay();
        vtcur = QString::number(tcur.elapsed());
        vt.append(vtcur);vt.append("ms ; ");
        myfile << vtcur.toUtf8().constData();myfile << "\n";
        if(ui->StopSimu->isDown())  break;
        cpt++;
    }
    double errpercoutliergomne = double(errgomne)/double(cpt);
    myfile.close();
    QString mess = "Execution time : ";
    double exec = tsimu.elapsed();
    mess.append(QString::number(exec));mess.append(" ms\n");
    mess.append(vt);mess.append(" ms\n");
    double cerpos=0;
    double i=0;
    vector<double> vcerpos;
    while (!errpos.empty()){
        cerpos+=errpos.back();
        i++;
        vcerpos.push_back(cerpos);
        errpos.pop_back();
    }
    double mean= cerpos/i;
    cerpos = 0;
    while(!vcerpos.empty()){
        cerpos+=pow(vcerpos.back()-mean,2);
        vcerpos.pop_back();
    }
    cerpos/=i;
    double area=0;
    for (size_t ii = 0; ii < par->ratio_area.size(); ii++){  area += par->ratio_area[ii];
    //cout<<par->ratio_area[ii]<<endl;
    }
    area/=par->ratio_area.size();
    cantlocalize/=cpt;cantlocalize*=100;

    mess.append(QString::number(cerpos));mess.append("\n");//mess.append(" variance (pixel)");
    mess.append(QString::number(mean));mess.append("\n");//mess.append(" average error (pixel)\n");
    mess.append(QString::number(exec));mess.append("\n");
    mess.append(QString::number(errpercoutliergomne));mess.append("\n");
    mess.append(QString::number(area));mess.append("\n");
    mess.append(QString::number(cantlocalize));mess.append("\n");
    QMessageBox::information(this,"End of Simulation",mess);
}

// Update current window and update certain parameter for the simulation
void MainWindow::repaint()
{
    RobotTraj();
    uint cpt=0;
    for(double i=0;i<6500-111;i=i+10){
        //R->DrawBox(xv[i]-0.01,xv[i]+0.01,yv[i]-0.01,yv[i]+0.01,QPen(traj_color(i)),QBrush(Qt::NoBrush));
        R->DrawLine(xv[i],yv[i],xv[i+10],yv[i+10],QPen(traj_color(i)));
        cpt++;
    }
    if(drawarrow==1)    R->DrawArrow(par->robot_position[0],par->robot_position[1],
                                     step*par->speedx[par->iteration-step],step*par->speedy[par->iteration-step]);
    R->DrawRobot(par->robot_position[0],par->robot_position[1],par->robot_position[3]);
    double xins=par->robot_position_found[0];
    double yins=par->robot_position_found[1];
    double zins=par->robot_position_found[2];
    if (drawapproxpos==1)   R->DrawRobot2(xins,yins,par->robot_position[3]);
    double errdist = sqrt(pow(xins-par->robot_position[0],2)+pow(yins-par->robot_position[1],2)+pow(zins-par->robot_position[2],2));
    if (isnan(errdist)==0){
        errpos.push_back(errdist);
    }
    else    cantlocalize++;
    ui->ratio_area_lcd->display(par->areain/(par->areain+par->areap)*100);
    par->ratio_area.push_back(par->areain/(par->areap+par->areain)*100);
    R->Save("paving");
}

void MainWindow::SLAM(int step){
    double xmin=100,xmax=-100,ymin=100,ymax=-100;
    if(par->iteration!=0){
        par->box.clear();
        while(par->vin_prev.empty()==false){
            Interval xtmp=par->vin_prev.back()[0];
            Interval ytmp=par->vin_prev.back()[1];
            //cout<<par->vin_prev.back()[0]<<endl;
            if(xtmp.lb()<xmin) xmin=xtmp.lb();
            if(ytmp.lb()<ymin) ymin=ytmp.lb();
            if(xtmp.ub()>xmax) xmax=xtmp.ub();
            if(ytmp.ub()>ymax) ymax=ytmp.ub();
            par->vin_prev.pop_back();
        }
        double dt = 0.02;
        IntervalVector ivtemp(2);
        double aprox=0.01+double(sqrt(step)/10);
        ivtemp[0]=Interval(xmin-aprox,xmax+aprox)+step*par->speedx[par->iteration-step];
        ivtemp[1]=Interval(ymin-aprox,ymax+aprox)+step*par->speedy[par->iteration-step];
        par->box.push_back(ivtemp);
        //cout<<"ivtemp"<<ivtemp[0]<<";"<<ivtemp[1]<<endl;
    }
}

// Call simulation gomne
void MainWindow::on_ButtonGOMNE_clicked()
{
    if (first_simu == false){
        QMessageBox::warning(this,"Abort process",
        "To run the alogrithm at a given time, first you need to run a simulation.\nYou can stop it before the end by holding the STOP button.\nYour current simulation will end.");
        Simu(2);
    }
    QElapsedTimer tgomne;
    tgomne.start();
    RobotTraj();
    Init();

    par->isinside=0;
    par->q = ui->BeaconSpinBox->value();

    par->nb_beacon = ui->BeaconSpinBox->value();
    //cout<<"q"<<par->q<<endl;
    ui->EpsilonSpinBox->setValue(1);
    for (uint i=0;i<100;i++){
       par->err[i] = 0.2;
    }
    ui->ErrSpinBox_1->setValue(par->err[0]);
    ui->ErrSpinBox_2->setValue(par->err[1]);
    ui->ErrSpinBox_3->setValue(par->err[2]);
    ui->ErrSpinBox_4->setValue(par->err[3]);
    ui->ErrSpinBox_5->setValue(par->err[4]);

    //epsilon<0.01 check is just to stop the algorithm when it doesnt find a solution to not freeze the window and block the algorithm.
    while(par->isinside!=1 && par->epsilon_sivia>0.01){
        if(par->in_perhaps==0){
            par->q--;
            ui->InterSpinBox->setValue(par->q);
            Sivia sivia(*R,par);
            //cout<<"q--"<<endl;
        }
        if(par->in_perhaps==1){
            par->epsilon_sivia/=2;
            Sivia sivia(*R,par);
            ui->EpsilonSpinBox->setValue(par->epsilon_sivia);
        }
    }
    repaint();

    if (timeinfo){
        QString mess = "Execution time : ";
        mess.append(QString::number(tgomne.elapsed()));mess.append(" ms");
        QMessageBox::information(this,"Info",mess);
    }
    ui->InterSpinBox->setValue(par->q);
}
void MainWindow::GOMNE_fixed_q()
{
    QElapsedTimer tgomne;
    tgomne.start();
    RobotTraj();
    Init();

    par->isinside=0;

    par->nb_beacon = ui->BeaconSpinBox->value();
    //cout<<"q"<<par->q<<endl;
    ui->EpsilonSpinBox->setValue(1);
    for (uint i=0;i<100;i++){
       par->err[i] = 0.2;
    }
    ui->ErrSpinBox_1->setValue(par->err[0]);
    ui->ErrSpinBox_2->setValue(par->err[1]);
    ui->ErrSpinBox_3->setValue(par->err[2]);
    ui->ErrSpinBox_4->setValue(par->err[3]);
    ui->ErrSpinBox_5->setValue(par->err[4]);

    //epsilon<0.01 check is just to stop the algorithm when it doesnt find a solution to not freeze the window
    while(par->isinside!=1 && par->epsilon_sivia>0.01){
        if(par->in_perhaps==0){
            par->q--;
            ui->InterSpinBox->setValue(par->q);
            Sivia sivia(*R,par);
            //cout<<"q--"<<endl;
        }
        if(par->in_perhaps==1){
            par->epsilon_sivia/=2;
            Sivia sivia(*R,par);
            ui->EpsilonSpinBox->setValue(par->epsilon_sivia);
        }
    }
    SLAM(step);
    repaint();

    if (timeinfo){
        QString mess = "Execution time : ";
        mess.append(QString::number(tgomne.elapsed()));mess.append(" ms");
        QMessageBox::information(this,"Info",mess);
    }
    ui->InterSpinBox->setValue(par->q);
}
// Call simulation 'Soft Constraints'
void MainWindow::on_ButtonFindSol_clicked()
{
    if (first_simu == false){
        QMessageBox::warning(this,"Abort process",
        "To run the alogrithm at a given time, first you need to run a simulation.\nYou can stop it before the end by holding the STOP button.\nYour current simulation will end.");
        Simu(1);
    }
    QElapsedTimer timer;
    timer.start();
    RobotTraj();
    Init();
    ui->EpsilonSpinBox->setValue(0.5);
    if(ui->BeaconSpinBox->value()!=5)
        QMessageBox::warning(this,"Attention","This method only works with 5 beacons. Now running simulation with 5 beacons.\n Press Ok to continue...");
    ui->BeaconSpinBox->setValue(5);
    ui->InterSpinBox->setValue(par->nb_beacon);

    for (uint i=0;i<(sizeof(par->err)/sizeof(*par->err));i++){
       par->err[i] = 0.00;
    }
    Sivia sivia(*R,par);
    uint i=0;
    //double startstep=0.05+floor(10*par->epsilon_sivia)/10-floor(10*par->epsilon_sivia)/20;
    double startstep=1;
    double pas = 0.5*(1+par->erroutlier/10);
    int nstep = 2;
    int stepctr=0;

    while(pas>0.05){
        int forw=0;
        int back=0;

        while(par->isinside==1){
            for (uint j=0;j<par->nb_beacon;j++){
                par->err[j]=startstep;
                if(i==j) par->err[j]=startstep-((stepctr+1))*pas;
            }

            Sivia sivia(*R,par);
            stepctr=(stepctr+1)%nstep;
            if (stepctr==0){
                i++;
                i = i % par->nb_beacon;
                if(i==0)    startstep=startstep-pas;
            }
            back++;
        }
       // qDebug()<<"err back: "<<"is "<<par->err[0]<<";"<<par->err[1]<<";"<<par->err[2]<<";"<<par->err[3]<<";"<<par->err[4]<<endl;

        while(par->isinside==0){

            for (uint j=0;j<par->nb_beacon;j++){
                par->err[j]=startstep;
                if(i==j) par->err[j]=startstep+((stepctr+1))*pas;
            }

            Sivia sivia(*R,par);
            stepctr=(stepctr+1)%nstep;
            if (stepctr==0){
                i++;
                i = i % par->nb_beacon;
                if(i==0)    startstep=startstep+pas;
            }
            forw++;
        }
        //qDebug()<<"par->err for: "<<"is "<<par->err[0]<<";"<<par->err[1]<<";"<<par->err[2]<<";"<<par->err[3]<<";"<<par->err[4]<<endl;
        if(back>forw)
            startstep/=0.5;
        else
            startstep*=0.5;
        //cout<<"pas "<<pas<<endl;
        pas/=2;

    }
    ui->ErrSpinBox_1->setValue(par->err[0]);
    ui->ErrSpinBox_2->setValue(par->err[1]);
    ui->ErrSpinBox_3->setValue(par->err[2]);
    ui->ErrSpinBox_4->setValue(par->err[3]);
    ui->ErrSpinBox_5->setValue(par->err[4]);
    repaint();

    if (timeinfo){
        QString mess = "Execution time : ";
        mess.append(QString::number(timer.elapsed()));mess.append(" ms");
        QMessageBox::information(this,"Info",mess);
    }
}

// Call SIVIA with the current parameters on the GUI
void MainWindow::on_ButtonStartParam_clicked()
{
    QElapsedTimer timer;
    timer.start();
    RobotTraj();
    Init();
    Sivia sivia(*R,par);
    R->DrawRobot(par->robot_position[0],par->robot_position[1],par->robot_position[3]);
    repaint();
    if (timeinfo){
        QString mess = "Execution time : ";
        mess.append(QString::number(timer.elapsed()));mess.append(" ms");
        QMessageBox::information(this,"Info",mess);
    }

}

void MainWindow::on_ErrSpinBox_1_valueChanged(double arg1)
{
    par->err[0] = arg1;
}

void MainWindow::on_ErrSpinBox_2_valueChanged(double arg1)
{
    par->err[1] = arg1;
}
void MainWindow::on_ErrSpinBox_3_valueChanged(double arg1)
{
    par->err[2] = arg1;
}
void MainWindow::on_ErrSpinBox_4_valueChanged(double arg1)
{
    par->err[3] = arg1;
}
void MainWindow::on_ErrSpinBox_5_valueChanged(double arg1)
{
    par->err[4] = arg1;
}
void MainWindow::on_InterSpinBox_valueChanged(int arg1)
{
    par->q = arg1;
}
void MainWindow::Zoom(int step)
{
    xmin = xv[t+step]-5;
    xmax = xv[t+step]+5;
    ymin = yv[t+step]-5;
    ymax = yv[t+step]+5;
}
void MainWindow::on_Zoomplus_clicked()
{
    xmin /= 2;
    xmax /= 2;
    ymin /= 2;
    ymax /= 2;

    R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    Sivia sivia(*R,par);
    repaint();
}

void MainWindow::on_Zoomminus_clicked()
{
    xmin *= 2;
    xmax *= 2;
    ymin *= 2;
    ymax *= 2;
    repaint();
    R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    Sivia sivia(*R,par);
    repaint();
}

void MainWindow::on_ZoomZone_clicked()
{
    xmin = par->robot_position[0]-5;
    xmax = par->robot_position[0]+5;
    ymin = par->robot_position[1]-5;
    ymax = par->robot_position[1]+5;
    repaint();
    R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    Sivia sivia(*R,par);
    repaint();
}

void MainWindow::on_ZoomReset_clicked()
{
    xmin = -25;
    xmax = 25;
    ymin = -25;
    ymax = 25;
    repaint();
    R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    Sivia sivia(*R,par);
    repaint();
}

void MainWindow::on_EpsilonSpinBox_valueChanged(double arg1)
{
    par->epsilon_sivia = arg1;
}

void MainWindow::on_BeaconSpinBox_valueChanged(int arg1)
{
    par->nb_beacon = arg1;
}

void MainWindow::on_checkBox_toggled(bool checked)
{
    if(checked) timeinfo=1;
    else timeinfo=0;
}

void MainWindow::on_TSlider_valueChanged(int value)
{
    t = double(value);
}

void MainWindow::on_errorOutlierSpinBox_valueChanged(double arg1)
{
    par->erroutlier = arg1;
}

void MainWindow::on_Tplot_clicked()
{
    drawarrow=0;Simu(1);
}

void MainWindow::on_Tplot_2_clicked()
{
    drawarrow=0;Simu(2);
}
void MainWindow::on_ButtonGOMNE_SLAM_clicked()
{
   drawarrow=1;Simu(3);
}
// Create a delay to give the GUI time to plot results
void MainWindow::delay()
{
    QTime dieTime= QTime::currentTime().addMSecs(10);
    while( QTime::currentTime() < dieTime )
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::on_probsensorisfalseSpinBox_valueChanged(double arg1)
{
    probsensorfalse = arg1;
}

void MainWindow::on_DrawRobotApproxcheckBox_toggled(bool checked)
{
    if (checked==true)
        drawapproxpos=1;
    else
        drawapproxpos=0;
}

void MainWindow::on_step_SpinBox_valueChanged(double arg1)
{
    step = arg1;
}

void MainWindow::on_BeaconPosSpinBox_valueChanged(double arg1)
{
    par->beacon_interval = arg1;
}
