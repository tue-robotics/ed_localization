#ifndef ED_LOCALIZATION_TRANSFORM_H_
#define ED_LOCALIZATION_TRANSFORM_H_

#include <geolib/datatypes.h>

class Transform
{

public:

    inline const geo::Transform2& matrix() const { return pose_; }
    inline double rotation() const { return rotation_; }
    inline const geo::Vec2& translation() const { return pose_.t; }

    inline void setTranslation(const geo::Vec2& v) { pose_.t = v; }
    inline void set(const geo::Transform2& p)
    {
        pose_ = p;
        rotation_ = p.rotation();
    }

    inline void setRotation(double rot)
    {
        pose_.setRotation(rot);
        rotation_ = rot;
    }

private:

    geo::Transform2 pose_;
    double rotation_;

};

#endif // ED_LOCALIZATION_TRANSFORM_H_
