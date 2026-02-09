from roboautotask.scripts import robo_reset



def main():
    robo_reset.logger.info('RoboAutoTask正在启动')
    robo_reset.reset()
    robo_reset.logger.info('RoboAutoTask启动完成')


if __name__ == "__main__":
    main()
