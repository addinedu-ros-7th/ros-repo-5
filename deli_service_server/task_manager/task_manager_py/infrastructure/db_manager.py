# task_manager_py/infrastructure/db_manager.py

import pymysql

class DBManager:
    def __init__(self, host, user, password, db):
        self.host = host
        self.user = user
        self.password = password
        self.db = db

    def execute_query(self, query, args=None):
        conn = pymysql.connect(
            host=self.host,
            user=self.user,
            password=self.password,
            db=self.db
        )
        try:
            with conn.cursor() as cursor:
                cursor.execute(query, args)
                result = cursor.fetchall()
            conn.commit()
        finally:
            conn.close()

        return result
