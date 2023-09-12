function [dcm_next_, vrp_next_, com_next_] = DCMTrackingControl(dt, b, w, dcm_, vrp_, com_,dcm_desired_)

vrp_next_ = (1 / (1-b)) * dcm_desired_ - (b/(1-b))*dcm_;

dcm_next_ = vrp_next_ + exp(w*dt)*(dcm_ - vrp_next_);

com_next_ = ((w*dt) / (1+w*dt)) * dcm_next_ + (1/(1+w*dt)) *com_;

end