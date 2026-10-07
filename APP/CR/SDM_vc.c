/**
 * @file SDM_vc.c
 * @brief 完整 Cosserat 静力平衡 (VC 式) 实现 — 照搬 tools/sdm_vc.py 数学.
 *
 * 状态 y=[p(3);R(9);v(3);u(3)]; 未知 x=[v0(3);u0(3)]∈R6; 打靶消末端残差.
 * 段内 ODE: [Kse+A, G; B, Kbt+H][v̇;u̇]=[d;c] (RK2 midpoint).
 * 段界(L1) 段A锚丝集中力/矩跳跃; 末端残差 = n(L)−Fσ2−F_tip, m(L)−Lσ2.
 * 重力世界 -z 经 Rᵀ 转体坐标; 背骨可弯不可伸用大 Kse.
 *
 * @date 2026-09-05
 */
#include "SDM_vc.h"
#include <math.h>
#include <string.h>

/* 全局丝截面角 (rad): 丝0..5 → 0,60,120,180,240,300° */
static const double ALPHA[6] = {
    0.0f, 1.0471975512f, 2.0943951024f, 3.1415926536f, 4.1887902048f, 5.2359877560f };
/* 锚丝: 段A {0,2,4} 止于 L1; 段B {1,3,5} 到末端 */
static const int SEG_A[3] = { 0, 2, 4 };
static const int SEG_B[3] = { 1, 3, 5 };
static const double L1 = 0.225f;
static const double L2 = 0.225f;
static const double L  = 0.45f;
static const double E3[3] = { 0, 0, 1 };

static void hat(const double v[3], double m[3][3])
{
    m[0][0]=0;    m[0][1]=-v[2]; m[0][2]= v[1];
    m[1][0]= v[2];m[1][1]=0;     m[1][2]=-v[0];
    m[2][0]=-v[1];m[2][1]= v[0]; m[2][2]=0;
}

static void m3_mul(const double A[3][3], const double B[3][3], double C[3][3])
{
    for (int i=0;i<3;i++) for (int j=0;j<3;j++){
        C[i][j]=A[i][0]*B[0][j]+A[i][1]*B[1][j]+A[i][2]*B[2][j];
    }
}

static void m3_apply(const double A[3][3], const double v[3], double o[3])
{
    o[0]=A[0][0]*v[0]+A[0][1]*v[1]+A[0][2]*v[2];
    o[1]=A[1][0]*v[0]+A[1][1]*v[1]+A[1][2]*v[2];
    o[2]=A[2][0]*v[0]+A[2][1]*v[1]+A[2][2]*v[2];
}

static void m3_applyT(const double A[3][3], const double v[3], double o[3])
{
    o[0]=A[0][0]*v[0]+A[1][0]*v[1]+A[2][0]*v[2];
    o[1]=A[0][1]*v[0]+A[1][1]*v[1]+A[2][1]*v[2];
    o[2]=A[0][2]*v[0]+A[1][2]*v[1]+A[2][2]*v[2];
}

/* solve 6×6: M x = b (高斯消元就地) */
static int solve6(double M[6][6], double b[6])
{
    for (int c=0;c<6;c++){
        int piv=c; double mx=fabs(M[c][c]);
        for (int r0=c+1;r0<6;r0++){ double v=fabs(M[r0][c]); if(v>mx){mx=v;piv=r0;} }
        if (mx<1e-18f) return -1;
        if (piv!=c){ for(int k=0;k<6;k++){double t=M[c][k];M[c][k]=M[piv][k];M[piv][k]=t;} double t=b[c];b[c]=b[piv];b[piv]=t; }
        double d=M[c][c];
        for (int k=0;k<6;k++) M[c][k]/=d;
        b[c]/=d;
        for (int r0=0;r0<6;r0++) if(r0!=c){ double f=M[r0][c]; for(int k=0;k<6;k++) M[r0][k]-=f*M[c][k]; b[r0]-=f*b[c]; }
    }
    return 0;
}

static void wire_r(const sdm_vc_params_t *p, int i, double r[3])
{
    r[0]=p->r_disk*cos(ALPHA[i]); r[1]=p->r_disk*sin(ALPHA[i]); r[2]=0;
}

/* intermedquant: 6 丝分布几何项, 得 A,B,G,H,c,d (照搬 Python) */
static void intermedquant(const double u[3], const double v[3],
                          const double Kse[3][3], const double Kbt[3][3],
                          const int *wires, int nw, const double tau[6],
                          const sdm_vc_params_t *p, const double fe_b[3],
                          double A[3][3], double B[3][3], double G[3][3],
                          double H[3][3], double c[3], double d[3])
{
    memset(A,0,9*sizeof(double)); memset(B,0,9*sizeof(double));
    memset(G,0,9*sizeof(double)); memset(H,0,9*sizeof(double));
    double a[3]={0,0,0}, b[3]={0,0,0};
    double uh[3][3]; hat(u, uh);
    for (int w=0;w<nw;w++){
        int i=wires[w]; double ri[3]; wire_r(p,i,ri);
        double uri[3]; m3_apply(uh, ri, uri);
        double pb[3]={ uri[0]+v[0], uri[1]+v[1], uri[2]+v[2] };
        double nrm=sqrt(pb[0]*pb[0]+pb[1]*pb[1]+pb[2]*pb[2]);
        if (nrm<1e-9f) continue;
        double pbhat[3][3]; hat(pb,pbhat);
        double pbhat2[3][3]; m3_mul(pbhat,pbhat,pbhat2);
        double coef=-(tau[i]/(nrm*nrm*nrm));
        double Ai[3][3];
        for(int r=0;r<3;r++)for(int q=0;q<3;q++) Ai[r][q]=coef*pbhat2[r][q];
        double rih[3][3]; hat(ri,rih);
        double Bi[3][3]; m3_mul(rih,Ai,Bi);
        double Airih[3][3]; m3_mul(Ai,rih,Airih);
        double Biri[3][3]; m3_mul(Bi,rih,Biri);
        for(int r=0;r<3;r++)for(int q=0;q<3;q++){
            A[r][q]+=Ai[r][q]; B[r][q]+=Bi[r][q];
            G[r][q]-=Airih[r][q]; H[r][q]-=Biri[r][q];
        }
        double uhat_pb[3]; m3_apply(uh,pb,uhat_pb);
        double ai[3]; m3_apply(Ai,uhat_pb,ai);
        double rih_ai[3]; m3_apply(rih,ai,rih_ai);
        for(int r=0;r<3;r++){ a[r]+=ai[r]; b[r]+=rih_ai[r]; }
    }
    /* c,d */
    double vv[3]={v[0]-E3[0],v[1]-E3[1],v[2]-E3[2]};
    double uhKbtu[3], vhKsev[3], uhKsev[3];
    double kbtu[3]; m3_apply(Kbt,u,kbtu); m3_apply(uh,kbtu,uhKbtu);
    double Ksev[3]; m3_apply(Kse,vv,Ksev);
    double vh[3][3]; hat(v,vh);
    m3_apply(vh,Ksev,vhKsev);
    m3_apply(uh,Ksev,uhKsev);
    for(int r=0;r<3;r++){
        c[r]=-uhKbtu[r]-vhKsev[r]-fe_b[r]-b[r];   /* le_b=0 */
        d[r]=-uhKsev[r]-fe_b[r]-a[r];
    }
}

/* 锚丝终止集中力/矩(空间系) */
static void anchor_loads(const double u[3], const double v[3], const double R[3][3],
                         const int *anchor, int na, const double tau[6],
                         const sdm_vc_params_t *p, double F[3], double Lm[3])
{
    F[0]=F[1]=F[2]=0; Lm[0]=Lm[1]=Lm[2]=0;
    double uh[3][3]; hat(u,uh);
    for (int w=0;w<na;w++){
        int i=anchor[w]; double ri[3]; wire_r(p,i,ri);
        double uri[3]; m3_apply(uh,ri,uri);
        double pb[3]={uri[0]+v[0],uri[1]+v[1],uri[2]+v[2]};
        double psp[3]; m3_apply(R,pb,psp);
        double nrm=sqrt(psp[0]*psp[0]+psp[1]*psp[1]+psp[2]*psp[2]);
        if (nrm<1e-9f) continue;
        double Rri[3]; m3_apply(R,ri,Rri);
        double Rrih[3][3]; hat(Rri,Rrih);
        double Rrih_psp[3]; m3_apply(Rrih,psp,Rrih_psp);
        for(int r=0;r<3;r++){
            F[r]-= tau[i]*psp[r]/nrm;
            Lm[r]-= tau[i]*Rrih_psp[r]/nrm;
        }
    }
}

/* 打靶前向: x=[v0;u0] → 末端残差 res[6]. (曲率/形状曲线暂不进 out, 可后续加) */
void vc_forward(const double x[6], const double tau[6],
                       const double F_tip[3], const double g_world[3],
                       const double R_mount[3][3], const sdm_vc_params_t *p,
                       double res[6], double tip[3])
{
    double v[3]={x[0],x[1],x[2]};
    double u[3]={x[3],x[4],x[5]};
    double R[3][3];
    if (R_mount) memcpy(R,R_mount,9*sizeof(double)); else { memset(R,0,9*sizeof(double)); R[0][0]=R[1][1]=R[2][2]=1; }
    double pos[3]={0,0,0};
    double Kse_end[3][3], Kbt_end[3][3];

    for (int k=0;k<2;k++){
        /* 段 k: 0=段A(两管叠), 1=段B */
        double Kse[3][3]={{0}}, Kbt[3][3]={{0}};
        double EA=p->EA, GA=p->GA, EI=p->EI, GJ=p->GJ;
        if (k==0){ Kse[0][0]=Kse[1][1]=2*GA; Kse[2][2]=2*EA;
                   Kbt[0][0]=Kbt[1][1]=2*EI; Kbt[2][2]=2*GJ; }
        else     { Kse[0][0]=Kse[1][1]=GA;   Kse[2][2]=EA;
                   Kbt[0][0]=Kbt[1][1]=EI;   Kbt[2][2]=GJ; }
        double Lseg = (k==0)?L1:L2;
        int nsub = (k==0)?p->n_p:p->n_d;
        double ds = Lseg/nsub;
        int wires[6]; int nw;
        if (k==0){ for(int i=0;i<6;i++)wires[i]=i; nw=6; }
        else     { for(int i=0;i<3;i++)wires[i]=SEG_B[i]; nw=3; }
        /* 该段分布重力(世界)转体 */
        double rho_total = p->rhoA + (p->disk_mass/L);  /* kg/m 折算盘重 */
        for (int j=0;j<nsub;j++){
            double fe_w[3]={0,0,rho_total*(-9.81f)};
            double fe_b[3]; m3_applyT(R,fe_w,fe_b);
            /* 中间点 RK2 */
            double A1[3][3],B1[3][3],G1[3][3],H1[3][3],c1[3],d1[3];
            intermedquant(u,v,Kse,Kbt,wires,nw,tau,p,fe_b,A1,B1,G1,H1,c1,d1);
            double M[6][6]; double rhs[6];
            for(int r=0;r<3;r++){ for(int q=0;q<3;q++){ M[r][q]=Kse[r][q]+A1[r][q]; M[r][q+3]=G1[r][q]; M[r+3][q]=B1[r][q]; M[r+3][q+3]=Kbt[r][q]+H1[r][q]; } rhs[r]=d1[r]; rhs[r+3]=c1[r]; }
            double vu[6]; memcpy(vu,rhs,6*sizeof(double));
            if (solve6(M,vu)) { /* singular */ }
            double vm[3],um[3];
            for(int r=0;r<3;r++){ vm[r]=v[r]+0.5f*ds*vu[r]; um[r]=u[r]+0.5f*ds*vu[r+3]; }
            double A2[3][3],B2[3][3],G2[3][3],H2[3][3],c2[3],d2[3];
            intermedquant(um,vm,Kse,Kbt,wires,nw,tau,p,fe_b,A2,B2,G2,H2,c2,d2);
            double M2[6][6]; double rhs2[6];
            for(int r=0;r<3;r++){ for(int q=0;q<3;q++){ M2[r][q]=Kse[r][q]+A2[r][q]; M2[r][q+3]=G2[r][q]; M2[r+3][q]=B2[r][q]; M2[r+3][q+3]=Kbt[r][q]+H2[r][q]; } rhs2[r]=d2[r]; rhs2[r+3]=c2[r]; }
            double vu2[6]; memcpy(vu2,rhs2,6*sizeof(double));
            if (solve6(M2,vu2)) { }
            for(int r=0;r<3;r++){ v[r]+=ds*vu2[r]; u[r]+=ds*vu2[r+3]; }
            /* p, R 更新 (中点位姿) */
            double uh[3][3]; hat(um,uh);
            /* Rmid = R*(I+0.5 ds uh) */
            double inc[3][3]={{1,0,0},{0,1,0},{0,0,1}};
            for(int r=0;r<3;r++)for(int q=0;q<3;q++) inc[r][q]+=0.5f*ds*uh[r][q];
            double Rm2[3][3]; m3_mul(R,inc,Rm2);
            double vmid[3]; m3_apply(Rm2,vm,vmid);
            for(int r=0;r<3;r++) pos[r]+=ds*vmid[r];
            double uhf[3][3]; hat(u,uhf);
            double Rf[3][3]; m3_mul(R,uhf,Rf);
            for(int r=0;r<3;r++)for(int q=0;q<3;q++) R[r][q]+=ds*Rf[r][q];
            /* 正交化 R (Gram-Schmidt on columns) */
            double cc0[3]={R[0][0],R[1][0],R[2][0]}, cc1[3]={R[0][1],R[1][1],R[2][1]};
            double l0=sqrt(cc0[0]*cc0[0]+cc0[1]*cc0[1]+cc0[2]*cc0[2]);
            if(l0>1e-12){cc0[0]/=l0;cc0[1]/=l0;cc0[2]/=l0;}
            double dd=cc0[0]*cc1[0]+cc0[1]*cc1[1]+cc0[2]*cc1[2];
            cc1[0]-=dd*cc0[0];cc1[1]-=dd*cc0[1];cc1[2]-=dd*cc0[2];
            double l1=sqrt(cc1[0]*cc1[0]+cc1[1]*cc1[1]+cc1[2]*cc1[2]);
            if(l1>1e-12){cc1[0]/=l1;cc1[1]/=l1;cc1[2]/=l1;}
            R[0][0]=cc0[0];R[1][0]=cc0[1];R[2][0]=cc0[2];
            R[0][1]=cc1[0];R[1][1]=cc1[1];R[2][1]=cc1[2];
            R[0][2]=cc0[1]*cc1[2]-cc0[2]*cc1[1];
            R[1][2]=cc0[2]*cc1[0]-cc0[0]*cc1[2];
            R[2][2]=cc0[0]*cc1[1]-cc0[1]*cc1[0];
        }
        memcpy(Kse_end,Kse,9*sizeof(double)); memcpy(Kbt_end,Kbt,9*sizeof(double));
        if (k==0){
            /* 段界跳跃: 段A 锚丝集中力 */
            double Fs[3],Ls[3];
            anchor_loads(u,v,R,SEG_A,3,tau,p,Fs,Ls);
            double RF[3], RL[3]; m3_applyT(R,Fs,RF); m3_applyT(R,Ls,RL);
            for(int r=0;r<3;r++){
                double inv=(r==2)?(1.0f/(2*EA)):(1.0f/(2*GA)); /* 段A Kse/2 */
                v[r]-=inv*RF[r];
                double invk=(r==2)?(1.0f/(2*GJ)):(1.0f/(2*EI));
                u[r]-=invk*RL[r];
            }
        }
    }
    /* 末端残差: n=R Kse_end(v-E3); m=R Kbt_end u; 减 段B 锚丝 + F_tip */
    double vv[3]={v[0],v[1],v[2]-1};
    double Kse_vv[3]; m3_apply(Kse_end,vv,Kse_vv);
    double n_e[3]; m3_apply(R,Kse_vv,n_e);
    double Kbt_u[3]; m3_apply(Kbt_end,u,Kbt_u);
    double m_e[3]; m3_apply(R,Kbt_u,m_e);
    double Fs[3],Ls[3];
    anchor_loads(u,v,R,SEG_B,3,tau,p,Fs,Ls);
    for(int r=0;r<3;r++){
        res[r]=n_e[r]-Fs[r]-(F_tip?F_tip[r]:0.0f);
        res[r+3]=m_e[r]-Ls[r];
    }
    tip[0]=pos[0];tip[1]=pos[1];tip[2]=pos[2];
}

uint8_t sdm_vc_solve_statics(const double tau[6],
                             const double F_tip[3],
                             const double g_world[3],
                             const double R_mount[3][3],
                             const sdm_vc_params_t *par,
                             sdm_vc_result_t *out)
{
    if (tau==NULL || par==NULL || out==NULL) return 1;
    double g[3]={0,0,-9.81f};
    if (g_world){ g[0]=g_world[0];g[1]=g_world[1];g[2]=g_world[2]; }

    double x[6]={0,0,1,0,0,0};   /* v0=(0,0,1), u0=0 */
    double res[6], tip[3];
    vc_forward(x,tau,F_tip,g,R_mount,par,res,tip);

    /* Levenberg 打靶 */
    int converged=0, it=0;
    double lam=1e-3f;
    for (it=0; it<60; it++){
        double J[6][6]; double h=1e-5f;
        for (int c=0;c<6;c++){
            double xp[6],xm[6],rp[6],rm[6],tp[3],tm[3];
            memcpy(xp,x,6*sizeof(double)); memcpy(xm,x,6*sizeof(double));
            xp[c]+=h; xm[c]-=h;
            vc_forward(xp,tau,F_tip,g,R_mount,par,rp,tp);
            vc_forward(xm,tau,F_tip,g,R_mount,par,rm,tm);
            for(int r=0;r<6;r++) J[r][c]=(rp[r]-rm[r])/(2*h);
        }
        double H[6][6], gt[6];
        for(int r=0;r<6;r++){ for(int q=0;q<6;q++){ double s=0; for(int k=0;k<6;k++)s+=J[k][r]*J[k][q]; H[r][q]=s; } H[r][r]+=lam; }
        for(int r=0;r<6;r++){ double s=0; for(int k=0;k<6;k++)s+=J[k][r]*res[k]; gt[r]=-s; }
        double dx[6]; memcpy(dx,gt,6*sizeof(double));
        if (solve6(H,dx)) break;
        /* 步长钳制: 防刚性系统牛顿跳跃震荡 */
        double dxmax=0; for(int r=0;r<6;r++){double a=fabs(dx[r]); if(a>dxmax)dxmax=a;}
        if (dxmax>0.5f){ for(int r=0;r<6;r++) dx[r]*=0.5f/dxmax; }
        double xnew[6]; for(int r=0;r<6;r++) xnew[r]=x[r]+dx[r];
        double res2[6],tip2[3];
        vc_forward(xnew,tau,F_tip,g,R_mount,par,res2,tip2);
        double nrm_old=0,nrm_new=0;
        for(int r=0;r<6;r++){ nrm_old+=res[r]*res[r]; nrm_new+=res2[r]*res2[r]; }
        if (nrm_new>=nrm_old && lam<1e6f){ lam*=10; continue; }
        lam=(lam/5.0f<1e-10f)?1e-10f:lam/5.0f;
        for(int r=0;r<6;r++) x[r]=xnew[r];
        for(int r=0;r<6;r++) res[r]=res2[r];
        if (nrm_new<1e-18f){ converged=1; break; }
        /* x 步长已小 → 形状收敛 (double 残差噪声层) */
        if (dxmax<1e-5f){ converged=1; break; }
        /* inf norm (double 刚性系统残差噪声下限 ~1e-3; 形状精度看 tip) */
        double in=0; for(int r=0;r<6;r++){double a=fabs(res[r]); if(a>in)in=a;}
        if (in < 8e-3f){ converged=1; break; }
    }
    /* 末次前向取 tip */
    vc_forward(x,tau,F_tip,g,R_mount,par,res,tip);
    out->v0[0]=x[0];out->v0[1]=x[1];out->v0[2]=x[2];
    out->u0[0]=x[3];out->u0[1]=x[4];out->u0[2]=x[5];
    double sq=0; for(int r=0;r<6;r++) sq+=res[r]*res[r];
    out->rms_res=sqrt(sq/6.0f);
    out->converged=(uint8_t)converged;
    out->n_iter=it+1;
    out->tip[0]=tip[0];out->tip[1]=tip[1];out->tip[2]=tip[2];
    return 0;
}

