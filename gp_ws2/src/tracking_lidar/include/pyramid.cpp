#include <pyramid.hpp>

bool TestPointInPyramid (const pcl::PointXYZI  pcpoint, Pyramid *pyramid ){

	double left= pyramid->p4(1)*pcpoint.x + pyramid->p4(2)*pcpoint.y+ pyramid->p4(3)*pcpoint.z;
	double right=pyramid->p2(1)*pcpoint.x + pyramid->p2(2)*pcpoint.y+ pyramid->p2(3)*pcpoint.z;
	double down=pyramid->p3(1)*pcpoint.x + pyramid->p3(2)*pcpoint.y+ pyramid->p3(3)*pcpoint.z;
	double up=pyramid->p1(1)*pcpoint.x + pyramid->p1(2)*pcpoint.y+ pyramid->p1(3)*pcpoint.z;




	if (  (left>0 && right<0)||( left<0 && right>0)) {
		return false;
	}
	if (  (down>0 && right<0)||( down<0 && right>0)) {
		return false;
	}
	if (  (up>0 && right<0)||( up<0 && right>0)) {
		return false;
	}

	if (  (up>0 && down<0)||( up<0 && down>0)) {
		return false;
	}

	return true;

}




Pyramid create_pyramids(Pyramid pyramid, Eigen::MatrixXd  bounding_box, Eigen::MatrixXd invcalib){

    pyramid.p3d1=invcalib*(Eigen::Vector3d(bounding_box(0, 0),bounding_box(0, 1),1));
    pyramid.p3d2=invcalib*(Eigen::Vector3d(bounding_box(1, 0),bounding_box(1, 1),1));
    pyramid.p3d3=invcalib*(Eigen::Vector3d(bounding_box(2, 0),bounding_box(2, 1),1));
    pyramid.p3d4=invcalib*(Eigen::Vector3d(bounding_box(3, 0),bounding_box(3, 1),1));


    pyramid.p1= pyramid.p3d1.cross3(pyramid.p3d2);
    pyramid.p2= pyramid.p3d2.cross3(pyramid.p3d3);
    pyramid.p3= pyramid.p3d3.cross3(pyramid.p3d4);
    pyramid.p4= pyramid.p3d4.cross3(pyramid.p3d1);

    return pyramid;

}
