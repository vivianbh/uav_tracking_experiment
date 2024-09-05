import if4nctu.job as Job


def cr_task(config_file, conn_list, icomm, freq=30):
    """
    :param config_file:
    :param conn_list:
    :param icomm:
    :param freq: HZ
    :return:
    """
    task=Job.IF_Job(config_file, conn_list, icomm, freq)
    return task