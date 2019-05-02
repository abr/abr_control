@pytest.mark.parametrize('q', (np.zeros(6), np.ones(6)*6.28))
@pytest.mark.parametrize('dq', (np.ones(6)*-2.5, np.ones(6)*2.5))
@pytest.mark.parametrize('in_index', (
    [0], [0, 1], [0, 1, 2], [0, 1, 2, 3], [0, 1, 2, 3, 4], [0, 1, 2, 3, 4, 5]))
def test_generate_scaled_inputs(q, dq, in_index):

    scaled_q, scaled_dq = network_utils.generate_scaled_inputs(
        q, dq, [in_index])

    assert np.all(0 <= scaled_q) and np.all(scaled_q  <= 1)
    assert np.all(0 <= scaled_dq) and np.all(scaled_dq <= 1)


