#ifndef BMI160_GYRO_HPP_
#define BMI160_GYRO_HPP_

#include <px4_config.h>

#include "bmi160.hpp"

/**
 * Helper class implementing the gyro driver node.
 */
class BMI160_gyro : public device::CDev
{
public:
	BMI160_gyro(BMI160 *parent, const char *path);
	~BMI160_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class BMI160;

	void			parent_poll_notify();

private:
	BMI160			*_parent;
	orb_advert_t		_gyro_topic;
	int			_gyro_orb_class_instance;
	int			_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	BMI160_gyro(const BMI160_gyro &);
	BMI160_gyro operator=(const BMI160_gyro &);
};




#endif /* BMI160_GYRO_HPP_ */
