#ifndef MOTOR_CONTROLLER_INTF_H
#define MOTOR_CONTROLLER_INTF_H

namespace auto_rover_base
{
    /** \brief Abstract base interface class for a motor controller
     * 
     * Inherit from this base class and specify the type of 
     * motor driver. The interface provides \ref setSpeed, which
     * is an abstract method and must therefore be implemented.
     */
    template<typename TMotorDriver>
    class MotorControllerIntf
    {
        public:

            /** \brief Set the speed of the left motor
             * 
             * Implement this method to set the speed of the left
             * motor that is connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param value positive or negative value to set the
             * direction and speed of the motor.
             * 
             */
            virtual void setMotorLeftSpeed(int16_t value) = 0;

            /** \brief Set the brake of the left motor
             * 
             * Implement this method to set the brake of the left
             * motor that is connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param value positive or negative brake value
             * 
             */
            virtual void setMotorLeftBrake(uint16_t value) = 0;

            /** \brief Set the speed of the right motor
             * 
             * Implement this method to set the speed of the right
             * motor that is connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param value positive or negative value to set the
             * direction and speed of the motor.
             * 
             */
            virtual void setMotorRightSpeed(int16_t value) = 0;

            /** \brief Set the brake of the right motor
             * 
             * Implement this method to set the brake of the right
             * motor that is connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param value positive or negative brake value
             * 
             */
            virtual void setMotorRightBrake(uint16_t value) = 0;

            /** \brief Set the speed of the left and right motors
             * 
             * Implement this method to set the speeds of both 
             * motors that are connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param left positive or negative value to set the
             * direction and speed of the left motor.
             * \param right positive or negative value to set the
             * direction and speed of the right motor.
             * 
             */
            virtual void setMotorSpeeds(int16_t left, int16_t right) = 0;

            /** \brief Set the brakes of the left and right motors
             * 
             * Implement this method to set the brake of both 
             * motors that are connected to the \ref motor_driver_
             * which is of type \ref TMotorDriver.
             * 
             * \param left positive value to set the brake power
             * of the left motor.
             * \param right positive value to set the brake power
             * of the right motor.
             * 
             */
            virtual void setMotorBrakes(uint16_t left, uint16_t right) = 0;

        protected:
            // Generic motor driver
            TMotorDriver motor_driver_;
    };
}

#endif // MOTOR_CONTROLLER_INTF_H