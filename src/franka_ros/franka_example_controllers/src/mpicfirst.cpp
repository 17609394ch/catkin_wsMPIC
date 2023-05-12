// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_example_controllers/mpicfirst.h"

#include "ros/ros.h"

void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}
MatrixXd kronProduct_(MatrixXd S,MatrixXd X){
    MatrixXd P(S.rows() * X.rows(), S.cols() * X.cols());
    P.setZero();
    for (int i = 0; i < S.rows(); i++)
    {
        for (int j = 0; j < S.cols(); j++)
        {
            P.block(i*X.rows(), j*X.cols(), X.rows(), X.cols()) = S(i, j) * X;
        }
    }
    return P;
}
MatrixXd diagMat_(MatrixXd diag, int shift){
    MatrixXd M (diag.size()+std::abs(shift),diag.size()+std::abs(shift));
    M.setZero();

    MatrixXd diag_res;
    if(diag.cols()<diag.rows()) diag_res = diag.transpose();
    else diag_res = diag;

    for (int i = 0; i < diag.size(); i++)
    {
        if(shift<0) M(i-shift,i)=diag_res(0,i);
        else M(i,i+shift)=diag_res(0,i);
    }
    return M;
}
MatrixXd powMat_(MatrixXd M, float pow){
    MatrixXd P = MatrixXd::Identity(M.rows(),M.cols());
    for(int i = 1;i<=pow;i++){
        P=P*M;
    }
    return P;
}
/*--------------------_MPIC--------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
_MPIC::_MPIC(int nx, int nu, int N)
    :_nx(nx)
    ,_nu(nu)
    ,_N(N)
{
    _selectX = MatrixXd::Zero(_nx,1);
    _selectu = MatrixXd::Zero(_nu,1);
    _Xmax = MatrixXd::Zero(_nx,1);
    _Xmin = MatrixXd::Zero(_nx,1);
    _umin = MatrixXd::Zero(_nu,1);
    _umax = MatrixXd::Zero(_nu,1);

    _A = MatrixXd::Zero(_nx,_nx);
    _B = MatrixXd::Zero(_nx,_nu);
    _K = MatrixXd::Zero(_nu,_nx);

    _nSlack = 1;
    _costSlack.push_back(1e16);
//    _costSlack.push_back(1e16);
//    _costSlack.push_back(1e16);
//    _costSlack.push_back(8e18);
//    _costSlack.push_back(8e18);
//    _costSlack.push_back(8e18);

//未验证
    QProblem qpb( _N*_nu + _nSlack, 2*_N*(_nx+_nu) + _nSlack);
    _qpb = qpb;
    Options options;
    options.setToMPC();
    options.printLevel = PL_NONE;
//    options.enableNZCTests = BT_TRUE;
//    options.enableFlippingBounds = BT_TRUE;
//    options.epsFlipping = 1e-6;
//    options.initialStatusBounds = ST_LOWER;
//    options.epsRegularisation = 1e-6;
    _qpb.setOptions( options );

    _uOpt = MatrixXd::Zero(_nu,1);
    _Xmodel = MatrixXd::Zero(_nx,1);
    _duOpt = MatrixXd::Zero(_nu,1);
    _qpSolution = new real_t[_nu*_N + _nSlack];

    _valueSlack = std::vector<double>(_nSlack);


    _QPfail = false;
}

_MPIC::~_MPIC(){
    delete[] _qpSolution;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::setSystem(MatrixXd A,MatrixXd B, MatrixXd K)
{
    _A = A;
    _B = B;
    _K = K;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::updateK(MatrixXd K)
{
    _K = K;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::computeQP(){
    /* weight */
    MatrixXd R = 10*MatrixXd::Identity(_nu,_nu);
    MatrixXd Q = _K.transpose()*R*_K;
    MatrixXd S = _K.transpose()*R;

    /* computing QP matrices */

    MatrixXd IN = MatrixXd::Identity(_N,_N);
    MatrixXd Qbar = kronProduct_(IN,Q);
    // Qbar.block(0,0,_nx,_nx) = MatrixXd::Zero(_nx,_nx);
    MatrixXd Rbar = kronProduct_(IN,R);
    MatrixXd Sbar = kronProduct_(IN,S);
    MatrixXd Cbar_tmp = kronProduct_(IN,_B);
    for(int i= 1;i<_N;i++){
        Cbar_tmp = Cbar_tmp + kronProduct_(diagMat_(MatrixXd::Ones(1,_N-i),-i),powMat_(_A,i)*_B);
    }
    MatrixXd Cbar = MatrixXd::Zero(Cbar_tmp.rows(),Cbar_tmp.cols());
    Cbar.bottomRightCorner(Cbar.rows()-_nx,Cbar.cols()-_nu) = Cbar_tmp.block(0,0,Cbar.rows()-_nx,Cbar.cols()-_nu);
    // ROS_INFO_STREAM( "Cbar"<<Cbar);B
    MatrixXd Abar = MatrixXd::Zero(_nx*_N,_nx);
    for(int i=0; i<_N;i++){
       Abar.block(_nx*i,0,_nx,_nx) = powMat_(_A,i);
    }
    // ROS_INFO_STREAM( "K"<<_K);
    // ROS_INFO_STREAM( "Q"<<Qbar);
    // ROS_INFO_STREAM( Rbar);
    // ROS_INFO_STREAM( Sbar);

    // ROS_INFO_STREAM( Abar);
    // ROS_INFO_STREAM( Cbar);
    MatrixXd Cw(_N*(_nx+_nu),_nu*_N);
    Cw.setZero();

    Cw.block(0,0,_N*_nx,Cw.cols()) = Cbar*kronProduct_(MatrixXd::Ones(_N,_N).triangularView<StrictlyLower>(),MatrixXd::Identity(_nu,_nu));
    Cw.block(_nx*_N,0,_N*_nu,Cw.cols()) = kronProduct_(MatrixXd::Ones(_N,_N).triangularView<StrictlyLower>(),MatrixXd::Identity(_nu,_nu));
    // ROS_INFO_STREAM("Cw"<< Cw);
    MatrixXd Aw(_N*(_nx+_nu),_nu+_nx);
    Aw.setZero();
    Aw.block(0,0,_N*_nx,_nx) = Abar;
    Aw.block(0,_nx,_N*_nx,_nu) = Cbar*kronProduct_(MatrixXd::Ones(_N,1),MatrixXd::Identity(_nu,_nu));
    Aw.block(_N*_nx,0,_N*_nu,_nx) = MatrixXd::Zero(_N*_nu,_nx);
    Aw.block(_N*_nx,_nx,_N*_nu,_nu) = kronProduct_(MatrixXd::Ones(_N,1),MatrixXd::Identity(_nu,_nu));
        // ROS_INFO_STREAM("Aw/n"<< Aw);
    MatrixXd Phi(_N*(_nu+_nx),_N*(3*_nu+2*_nx));
    Phi.setZero();
    Phi.block(0,0,_N*(_nu+_nx),_N*(_nu+_nx)) = MatrixXd::Identity(_N*(_nu+_nx),_N*(_nu+_nx));
    Phi.block(0,_N*(_nu+_nx),_N*(_nu+_nx),_N*_nu) = MatrixXd::Identity(_N*(_nu+_nx),_N*_nu);
    Phi.block(0,_N*(2*_nu+_nx),_N*(_nu+_nx),_N*(_nu+_nx)) = -MatrixXd::Identity(_N*(_nu+_nx),_N*(_nu+_nx));

    MatrixXd _Phi(_N*(_nu+_nx),_N*(4*_nu+2*_nx));
    _Phi.setZero();
    _Phi.block(0,0,_N*(_nu+_nx),_N*(_nu+_nx)) = MatrixXd::Identity(_N*(_nu+_nx),_N*(_nu+_nx));
    _Phi.block(0,_N*(_nu+_nx),_N*(2*_nu+_nx),_N*_nu+1) = MatrixXd::Identity(_N*(_nu+_nx),_N*_nu);
    _Phi.block(0,_N*(3*_nu+_nx),_N*(_nu+_nx),_N*(_nu+_nx)) = -MatrixXd::Identity(_N*(_nu+_nx),_N*(_nu+_nx));


    //  ROS_INFO_STREAM(_Phi);
    // ROS_INFO_STREAM(_Phi.rows());
    // ROS_INFO_STREAM(_Phi.cols());
    MatrixXd Psi(_N*(3*_nu+2*_nx),_N*_nu+(_N+1)*(_nu+_nx));
    Psi.setZero();
    Psi.block(0,0,_N*_nu,_N*_nu) = MatrixXd::Identity(_N*_nu,_N*_nu);
    Psi.block(_N*_nu,0,Cw.rows(),Cw.cols()) = Cw;
    Psi.block(_N*_nu,Cw.cols(),Aw.rows(),Aw.cols()) = Aw;
    Psi.block(_N*(2*_nu+_nx),_N*_nu+_nx+_nu,_N*(_nu+_nx),_N*(_nu+_nx)) = MatrixXd::Identity(_N*(_nx+_nu),_N*(_nx+_nu));

    MatrixXd TW(_nu*(_N+1),_nu);
    TW.setZero();
    for(int i=0; i<_N;i++){
       TW.block(_nu*(i+1),0,_nu,_nu) = MatrixXd::Identity(_nu,_nu);
    }

    // ROS_INFO_STREAM("TW"<<TW);
    MatrixXd _Psi(_N*(_nu*(9*_N+1)),_N*_nu+(_N+1)*(_nu+_nx));
    _Psi.setZero();
    _Psi.block(0,0,_N*_nu,_N*_nu) = MatrixXd::Identity(_N*_nu,_N*_nu);
    _Psi.block(_N*_nu,0,Cw.rows(),Cw.cols()) = Cw;
    _Psi.block((_N-1)*_nu+Cw.rows(),0,TW.rows(),TW.cols()) = TW;
    _Psi.block(_N*_nu,Cw.cols(),Aw.rows(),Aw.cols()) = Aw;
    _Psi.block(_N*(3*_nu+_nx),_N*_nu+_nx+_nu,_N*(_nu+_nx),_N*(_nu+_nx)) = MatrixXd::Identity(_N*(_nx+_nu),_N*(_nx+_nu));

    // ROS_INFO_STREAM(_Psi.rows());
    
    MatrixXd Q_(_N*(_nx+_nu),_N*(_nx+_nu));
    Q_.setZero();
    Q_.block(0,0,_N*_nu,_N*_nu) = Rbar;
    Q_.block(_N*_nu,0,_N*_nx,_N*_nu) = Sbar;
    Q_.block(0,_N*_nu,_N*_nu,_N*_nx) = Sbar.transpose();
    Q_.block(_N*_nu,_N*_nu,_N*_nx,_N*_nx) = Qbar;

    MatrixXd W_ = _Psi.transpose()*_Phi.transpose()*Q_*_Phi*_Psi;
    MatrixXd H_tmp = W_.block(0,0,_N*_nu,_N*_nu)/2+W_.block(0,0,_N*_nu,_N*_nu).transpose()/2;
    _F = W_.block(_N*_nu,0,(_N+1)*(_nu+_nx),_N*_nu);

    // ROS_INFO_STREAM(W_.rows());
    // ROS_INFO_STREAM(W_.cols());
    // softening constraints
    _H = Eigen::MatrixXd(H_tmp.rows()+_nSlack,H_tmp.cols()+_nSlack);

    for(int i=0;i<_nSlack;i++) _H(i,i) = _costSlack[i];
    _H.topRightCorner(_nSlack,H_tmp.cols()) = Eigen::MatrixXd::Zero(_nSlack,H_tmp.cols());
    _H.bottomLeftCorner(H_tmp.rows(),_nSlack) = Eigen::MatrixXd::Zero(H_tmp.rows(),_nSlack);
    _H.bottomRightCorner(H_tmp.rows(),H_tmp.cols()) = H_tmp;

    _H = (_H+_H.transpose())/2;


    /* Constraints */
    MatrixXd Ibar_tmp(_N*(_nu+_nx),_N*(_nu+_nx));
    Ibar_tmp.setZero();
    Ibar_tmp.block(0,0,_nx,_nx) = MatrixXd::Zero(_nx,_nx);
    Ibar_tmp.block(_nx,_nx,(_N-1)*_nx,(_N-1)*_nx) = kronProduct_(MatrixXd::Identity(_N-1,_N-1),diagMat_(_selectX,0));
    Ibar_tmp.block(_N*_nx,_N*_nx,_N*_nu,_N*_nu) = kronProduct_(MatrixXd::Identity(_N,_N),diagMat_(_selectu,0));
    // ROS_INFO_STREAM(Ibar_tmp);
    MatrixXd Ibar(2*_N*(_nu+_nx),_N*(_nu+_nx));
    MatrixXd Ibar_sel(2*_N*(_nu+_nx),2*_N*(_nu+_nx));
    Ibar << Ibar_tmp, -Ibar_tmp;
    Ibar_sel.topLeftCorner(Ibar_tmp.rows(),Ibar_tmp.cols()) = Ibar_tmp;
    Ibar_sel.topRightCorner(Ibar_tmp.rows(),Ibar_tmp.cols()) = Eigen::MatrixXd::Zero(Ibar_tmp.rows(),Ibar_tmp.cols());
    Ibar_sel.bottomLeftCorner(Ibar_tmp.rows(),Ibar_tmp.cols()) = Eigen::MatrixXd::Zero(Ibar_tmp.rows(),Ibar_tmp.cols());
    Ibar_sel.bottomRightCorner(Ibar_tmp.rows(),Ibar_tmp.cols()) = Ibar_tmp;
// ROS_INFO_STREAM(Ibar_sel);
    Eigen::MatrixXd Gp = Ibar*Cw;
    // ROS_INFO_STREAM("GP"<<Gp);
    _Gp = Eigen::MatrixXd(Gp.rows()+_nSlack,Gp.cols()+_nSlack);
    _Gp.topLeftCorner(_nSlack,_nSlack) = -Eigen::MatrixXd::Identity(_nSlack,_nSlack);
    _Gp.topRightCorner(_nSlack,Gp.cols()) = Eigen::MatrixXd::Zero(_nSlack,Gp.cols());
    Eigen::MatrixXd M = Ibar_sel*kronProduct_(Eigen::MatrixXd::Ones(Gp.rows()/_nSlack,1),-Eigen::MatrixXd::Identity(_nSlack,_nSlack));
    _Gp.bottomLeftCorner(Gp.rows(),_nSlack) = M;
    _Gp.bottomRightCorner(Gp.rows(),Gp.cols()) = Gp;
// ROS_INFO_STREAM("_Gp"<<_Gp);
// ROS_INFO_STREAM("_Gp"<<_Gp.rows());
// ROS_INFO_STREAM("_Gp"<<_Gp.cols());
    _Sp = -Ibar*Aw;

    _Wp_sup = MatrixXd::Zero(_N*(_nu+_nx),1);
    _Wp_inf = MatrixXd::Zero(_N*(_nu+_nx),1);

    _Wp_inf.block(0,0,_N*_nx,1) = kronProduct_(MatrixXd::Ones(_N,1),_Xmin);
    _Wp_inf.block(_N*_nx,0,_N*_nu,1) = kronProduct_(MatrixXd::Ones(_N,1),_umin);
    //    ROS_INFO_STREAM("_Wp_inf1"<<_Wp_inf); 
    _Wp_inf = Ibar_tmp*_Wp_inf;
    _Wp_sup.block(0,0,_N*_nx,1) = kronProduct_(MatrixXd::Ones(_N,1),_Xmax);
    _Wp_sup.block(_N*_nx,0,_N*_nu,1) = kronProduct_(MatrixXd::Ones(_N,1),_umax);
    _Wp_sup = Ibar_tmp*_Wp_sup;
//    ROS_INFO_STREAM("_Xmin"<<_Xmin); 
// ROS_INFO_STREAM("_Wp_inf"<<_Wp_inf);
// ROS_INFO_STREAM("_Wp_sup"<<_Wp_sup);
    _lb = MatrixXd::Zero(_Gp.rows()+_nSlack,1);
    _ub = MatrixXd::Zero(_Gp.rows()+_nSlack,1);
    _lb << 0.0*MatrixXd::Ones(_nSlack,1), -1*MatrixXd::Ones(_Gp.rows(),1);
    _ub << 10.0*MatrixXd::Ones(_nSlack,1), 1*MatrixXd::Ones(_Gp.rows(),1);

    _g_tmp = MatrixXd::Zero(1,(_N+1)*(_nx+_nu));
    _g = MatrixXd::Zero(1,_N*_nu+_nSlack);
    _W = MatrixXd::Zero(2*_Wp_sup.rows(),1);
    _ubA = MatrixXd::Zero(_W.rows()+_nSlack,1);

}
/*--------------------------------------------------------------------------------*/
void _MPIC::firstSolveMPIC(MatrixXd X0,MatrixXd r0){

    _QPfail = false;
    _g_tmp << X0.transpose(), _uOpt.transpose(),r0.transpose();
    _g << 0.0*MatrixXd::Ones(1,_nSlack), _g_tmp*_F;

    _W << _Wp_sup,-_Wp_inf;
    _ubA << 0.0*MatrixXd::Ones(_nSlack,1),_W+_Sp*_g_tmp.block(0,0,1,_nx+_nu).transpose();
    // ROS_INFO_STREAM(_ubA);
    // _Gp.block(_nSlack,1,_nu,_nu)=Eigen::MatrixXd::Identity(_nu,_nu);
    // _ubA.block(_nSlack,1,_nu,1)=-_uOpt-_B.inverse()*_A*X0;
//    _lb << 0.0,-100*MatrixXd::Ones(_Gp.rows(),1);
//    _ub << 10.0,100*MatrixXd::Ones(_Gp.rows(),1);

    int_t nWSR = 10000;

    try{
//        if(_qpb.init( _H.data(),_g.data(),_Gp.data(),_lb.data(),_ub.data(),nullptr,_ubA.data(), nWSR ) != SUCCESSFUL_RETURN) throw std::string("initial QP not solved");
        if(_qpb.init( _H.data(),_g.data(),_Gp.data(),nullptr,nullptr,nullptr,_ubA.data(), nWSR ) != SUCCESSFUL_RETURN) throw std::string("initial QP not solved");
    }
    catch(std::string e) {
        std::cerr << e << std::endl;
        _qpb.printProperties();
        _QPfail = true;
    }

//    _qpb.printProperties();
    _duOpt.setZero();

    if(!_QPfail){
        _qpb.getPrimalSolution( _qpSolution );
        for(int i=0;i<_nSlack;i++)
            _valueSlack[i] = _qpSolution[i];
        for(int i = 0; i<_nu; i++) _duOpt(i,0) = _qpSolution[i+_nSlack];
    }
    _uOpt+=_duOpt;
    _Xmodel = _A*X0+_B*_uOpt;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::updateSolveMPIC(MatrixXd X,MatrixXd rk){
    //softening constraints

    _QPfail = false;
    _g_tmp << X.transpose(), _uOpt.transpose(),rk.transpose();
    _g << 0.0*MatrixXd::Ones(1,_nSlack), _g_tmp*_F;

    _W << _Wp_sup,-_Wp_inf;
    _ubA << 0.0*MatrixXd::Ones(_nSlack,1),_W+_Sp*_g_tmp.block(0,0,1,_nx+_nu).transpose();
//    _Gp.block(_nSlack,1,_nu,_nu)=Eigen::MatrixXd::Identity(_nu,_nu);
    //  _Gp(1,2) = -1;
    //  _Gp(2,2) = 1;
    //  _Gp(3,2) = -_Ts;
    //  _Gp(4,2) = _Ts;
    // _Gp(5,3) = -_Ts*_Ts/2;
    // _Gp(4,2) = _Ts;
    Eigen::MatrixXd a;
    a = MatrixXd::Zero(_nx,1);
    Eigen::MatrixXd max=_K*(X-a.transpose())- _uOpt.transpose()+rk.block(0,0,1,_nu).transpose();
    // ROS_INFO_STREAM("1115"<<max);
    // ROS_INFO_STREAM("_Gp"<<_Gp);
    // _ubA(1,0)=0.5 + _uOpt(1);
    // _ubA(2,0)=0.5 - _uOpt(1);
    // _ubA(3,0)=0.08 + X(1);
    // _ubA(4,0)=0.08 - X(1);
    // _ubA(5,0)=0.088 + X(1)*_Ts  + X(7);
    // ROS_INFO_STREAM("ub"<<_ubA);
    Eigen::MatrixXd _B_pinv;
    pseudoInverse(_B, _B_pinv);
    // ROS_INFO_STREAM("112"<<_B);
    // _ubA.block(_nSlack,1,_nu,1)=-_uOpt-_B_pinv.block(5,0,_nx,1)*_A.block(5,0,_nx,1)*X;
    // ROS_INFO_STREAM("111"<<_A);
    // ROS_INFO_STREAM("111"<<_A.block(8,0,1,_nx)*X);

    _lb << 0.0*MatrixXd::Ones(_nSlack,1), -0.1*MatrixXd::Ones(_Gp.rows(),1);
    _ub << 10.0*MatrixXd::Ones(_nSlack,1), 0.1*MatrixXd::Ones(_Gp.rows(),1);

    int_t nWSR = 1000000000;

    try{
//        if(_qpb.hotstart(_g.data(),_lb.data(),_ub.data(),nullptr,_ubA.data(), nWSR )!= SUCCESSFUL_RETURN) throw std::string("hotstart QP not solved");
//        if(_qpb.hotstart(_g.data(),nullptr,nullptr,nullptr,_ubA.data(), nWSR )!= SUCCESSFUL_RETURN) throw std::string("hotstart QP not solved");
        // if(_qpb.init( _H.data(),_g.data(),_Gp.data(),_lb.data(),_ub.data(),nullptr,_ubA.data(), nWSR ) != SUCCESSFUL_RETURN) throw std::string("QP not solved");
       if(_qpb.init( _H.data(),_g.data(),_Gp.data(),nullptr,nullptr,nullptr,_ubA.data(), nWSR ) != SUCCESSFUL_RETURN) throw std::string("QP not solved");

    }
    catch(std::string e) {
        std::cerr << e << std::endl;
        _qpb.printProperties();
        _QPfail = true;
    }
    _duOpt.setZero();
    if(!_QPfail){
        _qpb.getPrimalSolution( _qpSolution );
        for(int i=0;i<_nSlack;i++)
            _valueSlack[i] = _qpSolution[i];
        for(int i = 0; i<_nu; i++) _duOpt(i) = _qpSolution[i+_nSlack];
    }
    _uOpt+=_duOpt;
    _Xmodel = _A*_Xmodel+_B*_uOpt;

}
/*--------------------------------------------------------------------------------*/
MatrixXd _MPIC::getuOpt(){
    return _uOpt;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::addConstraintsX(MatrixXd selectX, MatrixXd Xmax,MatrixXd Xmin){
    _selectX = selectX;
    _Xmax = Xmax;
    _Xmin = Xmin;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::addConstraintsU(MatrixXd selectu, MatrixXd umax,MatrixXd umin){
    _selectu = selectu;
    _umax = umax;
    _umin = umin;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::setHorizon(int N){
    _N = N;
}
/*--------------------------------------------------------------------------------*/
void _MPIC::setTimeStep(double Ts){
    _Ts = Ts;
}

/*--------------------------------------------------------------------------------*/
int _MPIC::getHorizon(){
    return _N;
}

/*--------------------------------------------------------------------------------*/
int _MPIC::getDimU(){
    return _nu;
}
/*--------------------------------------------------------------------------------*/
int _MPIC::getDimX(){
    return _nx;
}
/*--------------------------------------------------------------------------------*/
double _MPIC::getSlack(int j){
    return _valueSlack[j];
}
/*--------------------------------------------------------------------------------*/
MatrixXd _MPIC::getXmodel(){
    return _Xmodel;
}
