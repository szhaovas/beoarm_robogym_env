# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2 as robot__server__pb2


class RobotServerStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetState = channel.unary_unary(
                '/robot_server.RobotServer/GetState',
                request_serializer=robot__server__pb2.Empty.SerializeToString,
                response_deserializer=robot__server__pb2.State.FromString,
                )
        self.SetState = channel.unary_unary(
                '/robot_server.RobotServer/SetState',
                request_serializer=robot__server__pb2.State.SerializeToString,
                response_deserializer=robot__server__pb2.Success.FromString,
                )
        self.ResetState = channel.unary_unary(
                '/robot_server.RobotServer/ResetState',
                request_serializer=robot__server__pb2.State.SerializeToString,
                response_deserializer=robot__server__pb2.Success.FromString,
                )
        self.SendAction = channel.unary_unary(
                '/robot_server.RobotServer/SendAction',
                request_serializer=robot__server__pb2.Action.SerializeToString,
                response_deserializer=robot__server__pb2.Success.FromString,
                )
        self.SendActionGetState = channel.unary_unary(
                '/robot_server.RobotServer/SendActionGetState',
                request_serializer=robot__server__pb2.Action.SerializeToString,
                response_deserializer=robot__server__pb2.State.FromString,
                )
        self.SendGoalGetPlan = channel.unary_unary(
                '/robot_server.RobotServer/SendGoalGetPlan',
                request_serializer=robot__server__pb2.MoveitGoal.SerializeToString,
                response_deserializer=robot__server__pb2.MoveitPlan.FromString,
                )
        self.StrictActionGetState = channel.unary_unary(
                '/robot_server.RobotServer/StrictActionGetState',
                request_serializer=robot__server__pb2.Action.SerializeToString,
                response_deserializer=robot__server__pb2.State.FromString,
                )


class RobotServerServicer(object):
    """Missing associated documentation comment in .proto file."""

    def GetState(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SetState(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ResetState(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SendAction(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SendActionGetState(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SendGoalGetPlan(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def StrictActionGetState(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_RobotServerServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetState': grpc.unary_unary_rpc_method_handler(
                    servicer.GetState,
                    request_deserializer=robot__server__pb2.Empty.FromString,
                    response_serializer=robot__server__pb2.State.SerializeToString,
            ),
            'SetState': grpc.unary_unary_rpc_method_handler(
                    servicer.SetState,
                    request_deserializer=robot__server__pb2.State.FromString,
                    response_serializer=robot__server__pb2.Success.SerializeToString,
            ),
            'ResetState': grpc.unary_unary_rpc_method_handler(
                    servicer.ResetState,
                    request_deserializer=robot__server__pb2.State.FromString,
                    response_serializer=robot__server__pb2.Success.SerializeToString,
            ),
            'SendAction': grpc.unary_unary_rpc_method_handler(
                    servicer.SendAction,
                    request_deserializer=robot__server__pb2.Action.FromString,
                    response_serializer=robot__server__pb2.Success.SerializeToString,
            ),
            'SendActionGetState': grpc.unary_unary_rpc_method_handler(
                    servicer.SendActionGetState,
                    request_deserializer=robot__server__pb2.Action.FromString,
                    response_serializer=robot__server__pb2.State.SerializeToString,
            ),
            'SendGoalGetPlan': grpc.unary_unary_rpc_method_handler(
                    servicer.SendGoalGetPlan,
                    request_deserializer=robot__server__pb2.MoveitGoal.FromString,
                    response_serializer=robot__server__pb2.MoveitPlan.SerializeToString,
            ),
            'StrictActionGetState': grpc.unary_unary_rpc_method_handler(
                    servicer.StrictActionGetState,
                    request_deserializer=robot__server__pb2.Action.FromString,
                    response_serializer=robot__server__pb2.State.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'robot_server.RobotServer', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class RobotServer(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def GetState(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/GetState',
            robot__server__pb2.Empty.SerializeToString,
            robot__server__pb2.State.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SetState(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/SetState',
            robot__server__pb2.State.SerializeToString,
            robot__server__pb2.Success.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def ResetState(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/ResetState',
            robot__server__pb2.State.SerializeToString,
            robot__server__pb2.Success.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SendAction(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/SendAction',
            robot__server__pb2.Action.SerializeToString,
            robot__server__pb2.Success.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SendActionGetState(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/SendActionGetState',
            robot__server__pb2.Action.SerializeToString,
            robot__server__pb2.State.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SendGoalGetPlan(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/SendGoalGetPlan',
            robot__server__pb2.MoveitGoal.SerializeToString,
            robot__server__pb2.MoveitPlan.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def StrictActionGetState(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/robot_server.RobotServer/StrictActionGetState',
            robot__server__pb2.Action.SerializeToString,
            robot__server__pb2.State.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
