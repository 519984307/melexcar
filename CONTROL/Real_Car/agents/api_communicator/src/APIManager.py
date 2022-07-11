import json

class APIManager():
    def __int__(self, melex_id):
        self.Melex_ID = melex_id
        self.request = {
            'url': 'http://213.97.17.253:9000/requests',
            'json': None
        }
        self.put = {
            'url': 'http://213.97.17.253:9000/parametersCA/',
            'json': None
        }

    def __del__(self):
        pass

    def request_api(self):
        pass

    def put_parameters_api(self):
        pass

    def put_state_api(self):
        pass

    def create_request_json(self):
        pass

    def create_put_json(self, data):
        self.put["json"] = json.dumps(data)
