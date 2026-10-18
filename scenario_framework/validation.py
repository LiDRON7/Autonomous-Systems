"""Conservative execution checks; full trajectory/arrival validation belongs to #65."""
from .contracts import outcome


class Validator:
    @staticmethod
    def evaluate(config, events, execution_status):
        if execution_status != 'COMPLETED':
            raise ValueError('execution errors must preserve their original outcome')
        states = [e for e in events if e['kind'] == 'state']
        forbidden = [e['seq'] for e in states
                     if e['payload']['current'] in config['expected']['forbidden_states']]
        checks = [{'name': 'forbidden_states', 'passed': not bool(forbidden), 'evidence_seq': forbidden}]
        if forbidden:
            result = outcome('COMPLETED', 'EXPECTATION_FAILED', 'validation')
        else:
            # Receiving COMPLETE alone cannot establish path, arrival or cross-topic causality.
            # Keep this explicit until #65 supplies a validator for those predicates.
            result = outcome('ERROR', 'MONITOR_DATA_MISSING', 'validation',
                             'Detailed state/path/arrival validation is not implemented in #64; run is unverified.')
            for name in ('state_sequence', 'path', 'arrival'):
                checks.append({'name': name, 'passed': False, 'evidence_seq': [],
                               'message': 'Not evaluated; requires #65 validator and causal evidence.'})
        result['expectation_checks'] = checks
        return result
