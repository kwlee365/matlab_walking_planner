function [cp_next_, zmp_next_, com_next_] = CPTrackingControl(dt, b, w, cp_, zmp_, com_,cp_desired_)

zmp_next_ = (1 / (1-b)) * cp_desired_ - (b/(1-b))*cp_;

cp_next_ = zmp_next_ + exp(w*dt)*(cp_ - zmp_next_);

% com_next_ = ((w*dt) / (1+w*dt)) * cp_next_ + (1/(1+w*dt)) *com_;
com_next_ = cp_next_ + exp(-w*dt)*(com_ - cp_next_);

end