"""WIP."""

import base64

import requests

from srv_vision.secrets import Credentials_aws


class Client(Credentials_aws):
    """Class owner of aws connection."""

    headers = {"Content-Type": "application/json"}

    def __init__(self):
        """You could potentially validate the endpoint format or existence here."""
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        """WIP."""
        return True

    def _is_server_reachable(self) -> bool:
        """Implement logic to verify if self._enpoint is reachable."""
        # self._endpoint
        return True

    def _save_on_error(self):
        """This function should save a request if not reachable the destination."""
        # on case, receive an update of the post, remove pending post.
        pass

    def post(self, sender: dict[str, any]) -> str:
        """WIP."""
        try:
            response = requests.post(
                base64.b64decode(self._endpoint_s1 + self._endpoint_s2).decode("utf-8"),
                json=sender,
                headers=self.headers,
            )
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            if hasattr(e, "response"):
                return f"Error: code {e.response.status_code} received"
            return "Error: Unreachable server"
        return f"Ok: code {response.status_code} received"
