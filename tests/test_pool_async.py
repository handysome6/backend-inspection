import asyncio
import concurrent
import threading

from time import sleep


def hard_work(arg):
    print(arg)
    sleep(1)
    return 'hard work done!'


async def do_async_job(loop, executor):
    a = await loop.run_in_executor(executor, hard_work, 1)
    print(a)
    print('job done!')
    return a


async def main():
    loop = asyncio.get_event_loop()
    with concurrent.futures.ProcessPoolExecutor() as executor:
        task1 = asyncio.create_task(do_async_job(loop, executor))
        task2 = asyncio.create_task(do_async_job(loop, executor))
        task3 = asyncio.create_task(do_async_job(loop, executor))
        await asyncio.gather(task1, task2, task3)
        print(task1.result())


if __name__ == "__main__":
    asyncio.run(main())