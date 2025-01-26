# task_manager_py/infrastructure/db_manager.py

import pymysql

class DBManager:
    def __init__(self, host, user, password, db):
        self.conn = pymysql.connect(
            host=host, 
            user=user, 
            password=password, 
            db=db, 
            charset='utf8'
        )

    def execute_query(self, query, args=None):
        """
        SELECT, UPDATE, INSERT 등 모든 쿼리에 사용.
        결과가 필요한 경우에는 fetchall() 결과를 리턴.
        """
        with self.conn.cursor() as cursor:
            cursor.execute(query, args)
            self.conn.commit()
            return cursor.fetchall()

    def close(self):
        self.conn.close()

